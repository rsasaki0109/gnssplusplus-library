#!/usr/bin/env python3
"""Run the single sealed holdout evaluation for the multi-phone WLS lane.

The freeze record and its hash manifest are verified before any holdout
payload is opened.  The holdout device GNSS members are then materialized,
each phone's truth-free WLS and the frozen coordinate-wise ECEF median lane
are published atomically, and all artifacts are hashed.  Only after that
seal are the three ground-truth members opened once for one scoring pass.
This command has no tuning or rerun path.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import resource
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_multi_phone_ensemble_eval as ensemble  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-multi-phone-ensemble-holdout-evaluation.v1"
FREEZE_SCHEMA_VERSION = "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze.v1.4"
FREEZE_MANIFEST_SCHEMA_VERSION = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-freeze-manifest.v1.4"
)
RUN_MANIFEST_SCHEMA_VERSION = (
    "smartphone-r5-wls-multi-phone-ensemble-holdout-run-manifest.v1"
)
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_SELECTION = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_selection.json"
)
DEFAULT_FREEZE = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4.json"
)
DEFAULT_FREEZE_MANIFEST = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_multi_phone_ensemble_holdout_freeze_v1_4_manifest.json"
)
DEFAULT_OUTPUT = ROOT / "output" / "smartphone-r5" / "wls-multi-phone-ensemble-holdout-v1.4"
HOLDOUT_ROUTE = "2022-10-06-21-51-us-ca-mtv-n"
SELECTED_METHOD = "coordinate_wise_ecef_median"
BASELINE_METHOD = ensemble.BASELINE_METHOD_ID
ALIGNMENT_TOLERANCE_MS = ensemble.V1_4_ALIGNMENT_TOLERANCE_MS
MINIMUM_ALIGNED_PHONE_COUNT = 1
SIGNOFF_THRESHOLDS: dict[str, float] = {
    "availability_min": 0.98,
    "truth_coverage_min": 0.98,
    "horizontal_median_max_m": 7.0,
    "horizontal_p95_max_m": 25.0,
    "vertical_p95_max_m": 45.0,
}


class HoldoutEnsembleError(ValueError):
    """Raised when the frozen holdout contract cannot be proven."""


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise HoldoutEnsembleError(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise HoldoutEnsembleError(f"{label} must be a JSON object")
    return payload


def _sha256(path: Path) -> str:
    try:
        return ensemble._sha256(path)
    except ensemble.EnsembleError as exc:
        raise HoldoutEnsembleError(str(exc)) from exc


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    ensemble._atomic_json(path, payload)


def _relative_path(value: str) -> Path:
    path = Path(value)
    if path.is_absolute() or ".." in path.parts:
        raise HoldoutEnsembleError(f"freeze path must be repository-relative: {value}")
    return ROOT / path


def frozen_contract() -> dict[str, Any]:
    """Return the exact pre-holdout contract used by freeze and evaluator."""

    return {
        "route": HOLDOUT_ROUTE,
        "selected_method": SELECTED_METHOD,
        "baseline_method": BASELINE_METHOD,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "alignment_tolerance_ms": ALIGNMENT_TOLERANCE_MS,
        "minimum_aligned_phone_count": MINIMUM_ALIGNED_PHONE_COUNT,
        "fallback_policy": (
            "when fewer than two phones align, use the target phone's finite raw WLS row; "
            "record the one-phone fallback count and never synthesize a missing row"
        ),
        "wls_integrity": {
            "timestamp_gap_threshold_ms": wls.TIMESTAMP_GAP_THRESHOLD_MS,
            "continuous_condition": "elapsed_ms <= 1500",
            "gap_reset_condition": "elapsed_ms > 1500",
            "missing_partial_nonfinite_policy": "fail-closed",
            "all_epoch_rows_consistent": True,
            "all_selected_rows_finite": True,
            "allow_missing_wls_epochs": True,
            "allow_timestamp_gaps_as_diagnostic": True,
            "sparse_wls_policy": "retain epochs with at least one complete finite WLS row; omit all-blank epochs with timestamp/reason",
            "blank_row_policy": "count blank rows",
            "partial_coordinate_triplet_policy": "fail-closed",
            "nonfinite_coordinate_policy": "fail-closed",
            "inconsistent_coordinate_policy": "fail-closed",
            "extrapolation_policy": "forbidden",
        },
        "atomic_key_contract": {
            "publish": "fsync temporary file, replace destination, fsync parent directory",
            "submission_fields": list(kaggle.SUBMISSION_FIELDS),
            "key": "phone + exact UnixTimeMillis row identity",
            "combined_sort": "phone, then UnixTimeMillis",
            "partial_publish": "forbidden on integrity or non-finite failure",
        },
        "official_diagnostics": list(ensemble.DIAGNOSTIC_KEYS),
        "signoff_thresholds": dict(SIGNOFF_THRESHOLDS),
        "no_post_holdout_tuning": True,
        "truth_free_before_truth": True,
        "truth_evaluation_pass_count": 1,
    }


def _verify_freeze(
    freeze_path: Path,
    freeze_manifest_path: Path,
    selection_path: Path,
    selection: dict[str, Any],
    archive_hash: str,
    inventory_hash: str,
) -> tuple[dict[str, Any], str, str]:
    freeze = _load_json(freeze_path, "multi-phone holdout freeze")
    if freeze.get("schema_version") != FREEZE_SCHEMA_VERSION:
        raise HoldoutEnsembleError("holdout freeze schema is invalid")
    if freeze.get("status") != "frozen-before-holdout-payload-access":
        raise HoldoutEnsembleError("holdout freeze is not sealed before payload access")
    contract = freeze.get("contract")
    if contract != frozen_contract():
        raise HoldoutEnsembleError("holdout freeze contract differs from evaluator contract")
    selection_contract = freeze.get("selection")
    if not isinstance(selection_contract, dict):
        raise HoldoutEnsembleError("holdout freeze lacks selection hashes")
    if selection_contract.get("sha256") != _sha256(selection_path):
        raise HoldoutEnsembleError("holdout freeze selection hash differs")
    if selection_contract.get("sha256") != freeze.get("selection_record_sha256"):
        raise HoldoutEnsembleError("holdout freeze selection hash is not self-consistent")
    if freeze.get("archive_sha256") != archive_hash:
        raise HoldoutEnsembleError("holdout freeze archive hash differs")
    if freeze.get("inventory_sha256") != inventory_hash:
        raise HoldoutEnsembleError("holdout freeze inventory hash differs")
    if freeze.get("holdout_route") != HOLDOUT_ROUTE:
        raise HoldoutEnsembleError("holdout freeze route differs")
    selected_holdout = selection.get("next_holdout")
    frozen_holdout = freeze.get("holdout")
    if not isinstance(selected_holdout, dict) or not isinstance(frozen_holdout, dict):
        raise HoldoutEnsembleError("holdout freeze lacks role metadata")
    if frozen_holdout.get("route") != selected_holdout.get("route"):
        raise HoldoutEnsembleError("holdout freeze role differs from selection")
    if frozen_holdout.get("phones") != selected_holdout.get("phones"):
        raise HoldoutEnsembleError("holdout freeze phone list differs from selection")
    if frozen_holdout.get("records") != selected_holdout.get("records"):
        raise HoldoutEnsembleError("holdout freeze central metadata differs from selection")
    if frozen_holdout.get("materialization_forbidden_before_freeze") is not True:
        raise HoldoutEnsembleError("holdout freeze does not prove pre-freeze sealing")
    if frozen_holdout.get("truth_open_forbidden_before_freeze") is not True:
        raise HoldoutEnsembleError("holdout freeze does not prove pre-freeze truth sealing")
    if freeze.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise HoldoutEnsembleError("holdout freeze algorithm parameter hash differs")
    if freeze.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise HoldoutEnsembleError("holdout freeze algorithm core hash differs")
    execution_contract = freeze.get("holdout_execution_contract")
    if not isinstance(execution_contract, dict):
        raise HoldoutEnsembleError("holdout freeze lacks execution authorization contract")
    if execution_contract.get("authorized") is not True:
        raise HoldoutEnsembleError("holdout freeze execution authorization is absent")
    if execution_contract.get("truth_free_phase") is not True:
        raise HoldoutEnsembleError("holdout freeze execution is not truth-free")
    if execution_contract.get("no_post_holdout_tuning") is not True:
        raise HoldoutEnsembleError("holdout freeze execution lacks no-post-tuning guard")
    if execution_contract.get("route") != HOLDOUT_ROUTE:
        raise HoldoutEnsembleError("holdout freeze execution route differs")
    if execution_contract.get("phone_allowlist") != frozen_holdout.get("phones"):
        raise HoldoutEnsembleError("holdout freeze execution phone allowlist differs")
    if execution_contract.get("algorithm_parameter_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH:
        raise HoldoutEnsembleError("holdout freeze execution algorithm hash differs")
    if execution_contract.get("algorithm_core_hash") != wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH:
        raise HoldoutEnsembleError("holdout freeze execution algorithm core hash differs")
    if execution_contract.get("allow_missing_wls_epochs") is not True:
        raise HoldoutEnsembleError("holdout freeze execution sparse WLS authorization is absent")
    if execution_contract.get("allow_timestamp_gaps_as_diagnostic") is not True:
        raise HoldoutEnsembleError(
            "holdout freeze execution timestamp-gap diagnostic authorization is absent"
        )
    if execution_contract.get("extrapolation_policy") != "forbidden":
        raise HoldoutEnsembleError("holdout freeze execution does not forbid extrapolation")
    sparse_contract = freeze.get("sparse_wls_contract")
    if not isinstance(sparse_contract, dict) or sparse_contract.get(
        "allow_missing_wls_epochs"
    ) is not True:
        raise HoldoutEnsembleError("holdout freeze sparse WLS authorization is absent")
    if sparse_contract.get("single_phone_default_fail_closed") is not True:
        raise HoldoutEnsembleError("holdout freeze weakens single-phone default")
    if sparse_contract.get("allow_timestamp_gaps_as_diagnostic") is not True:
        raise HoldoutEnsembleError(
            "holdout freeze timestamp-gap diagnostic authorization is absent"
        )
    if sparse_contract.get("extrapolation_policy") != "forbidden":
        raise HoldoutEnsembleError("holdout freeze does not forbid extrapolation")
    source_hashes = freeze.get("source_hashes")
    if not isinstance(source_hashes, dict) or not source_hashes:
        raise HoldoutEnsembleError("holdout freeze lacks source hashes")
    for relative, expected in source_hashes.items():
        if not isinstance(relative, str) or not isinstance(expected, str):
            raise HoldoutEnsembleError("holdout freeze source hash shape is invalid")
        actual = _sha256(_relative_path(relative))
        if actual != expected:
            raise HoldoutEnsembleError(f"frozen source hash differs: {relative}")
    binary = freeze.get("release_binary")
    profile = freeze.get("profile")
    if not isinstance(binary, dict) or not isinstance(profile, dict):
        raise HoldoutEnsembleError("holdout freeze lacks binary/profile hashes")
    for label, row in (("Release binary", binary), ("profile", profile)):
        relative = row.get("path")
        expected = row.get("sha256")
        if not isinstance(relative, str) or not isinstance(expected, str):
            raise HoldoutEnsembleError(f"{label} hash shape is invalid")
        if _sha256(_relative_path(relative)) != expected:
            raise HoldoutEnsembleError(f"frozen {label} hash differs")
    freeze_hash = _sha256(freeze_path)
    freeze_manifest = _load_json(freeze_manifest_path, "holdout freeze manifest")
    if freeze_manifest.get("schema_version") != FREEZE_MANIFEST_SCHEMA_VERSION:
        raise HoldoutEnsembleError("holdout freeze manifest schema is invalid")
    manifest_record = freeze_manifest.get("freeze_record")
    if not isinstance(manifest_record, dict) or manifest_record.get("sha256") != freeze_hash:
        raise HoldoutEnsembleError("holdout freeze manifest does not pin freeze record hash")
    return freeze, freeze_hash, _sha256(freeze_manifest_path)


def _verify_holdout_central_metadata(
    archive_path: Path, freeze: dict[str, Any]
) -> None:
    """Check only ZIP central metadata for the frozen holdout."""

    holdout = freeze["holdout"]
    nav = holdout.get("central_directory_broadcast_nav")
    if not isinstance(nav, dict) or not isinstance(nav.get("name"), str):
        raise HoldoutEnsembleError("holdout freeze lacks navigation central metadata")
    if ensemble._central_metadata(archive_path, nav["name"]) != nav:
        raise HoldoutEnsembleError("holdout navigation central metadata changed")
    for record in holdout["records"]:
        dataset_id = str(record["dataset_id"])
        names = ensemble._member_names(dataset_id)
        for field, member_key in (("device_gnss", "device_gnss"), ("ground_truth", "ground_truth")):
            expected = record.get(field)
            if not isinstance(expected, dict) or ensemble._central_metadata(archive_path, names[member_key]) != expected:
                raise HoldoutEnsembleError(f"holdout {field} central metadata changed: {dataset_id}")


def _verify_reused_device_materialization(
    input_root: Path, freeze: dict[str, Any], holdout: dict[str, Any]
) -> dict[str, dict[str, Any]]:
    """Verify an already sealed device-only materialization before reuse."""

    frozen = freeze.get("reused_device_materialization")
    if not isinstance(frozen, dict):
        raise HoldoutEnsembleError("freeze lacks reused device materialization hashes")
    result: dict[str, dict[str, Any]] = {}
    for record in holdout["records"]:
        dataset_id = str(record["dataset_id"])
        phone = dataset_id.split("/", 1)[1]
        source = input_root / phone / "device_gnss.csv"
        expected = frozen.get(phone)
        if not isinstance(expected, dict) or not isinstance(expected.get("sha256"), str):
            raise HoldoutEnsembleError(f"freeze lacks reused device hash for {phone}")
        if not source.is_file() or source.stat().st_size != int(record["device_gnss"]["file_size"]):
            raise HoldoutEnsembleError(f"reused device materialization is invalid: {source}")
        artifact = _artifact(source)
        if artifact["sha256"] != expected["sha256"]:
            raise HoldoutEnsembleError(f"reused device hash differs for {phone}")
        result[phone] = {
            "member": ensemble._member_names(dataset_id)["device_gnss"],
            "metadata": record["device_gnss"],
            **artifact,
            "reused_existing_materialization": True,
        }
    if sorted(result) != sorted(holdout["phones"]):
        raise HoldoutEnsembleError("reused device phone allowlist differs")
    return result


def _verify_wls_integrity(
    wls_artifacts: dict[str, Any],
) -> dict[str, Any]:
    summaries: dict[str, Any] = {}
    for phone, artifacts in sorted(wls_artifacts.items()):
        summary_path = Path(artifacts["summary"]["path"])
        manifest_path = Path(artifacts["manifest"]["path"])
        summary = _load_json(summary_path, f"WLS summary for {phone}")
        manifest = _load_json(manifest_path, f"WLS manifest for {phone}")
        validation = summary.get("validation")
        populations = summary.get("populations")
        classification_counts = (
            validation.get("classification_counts")
            if isinstance(validation, dict)
            else None
        )
        manifest_contract = manifest.get("contract")
        manifest_sparse_allowed = (
            manifest_contract.get("allow_missing_wls_epochs") is True
            if isinstance(manifest_contract, dict)
            else False
        )
        sparse_allowed = (
            validation.get("allow_missing_wls_epochs") is True
            if isinstance(validation, dict)
            else False
        )
        gap_count = (
            populations.get("timestamp_gap_count_gt_1500ms")
            if isinstance(populations, dict)
            else None
        )
        gap_count_valid = (
            isinstance(gap_count, int)
            and not isinstance(gap_count, bool)
            and gap_count >= 0
        )
        gap_diagnostic_allowed = (
            validation.get("timestamp_gap_allowed_as_diagnostic") is True
            and manifest_contract.get("timestamp_gap_allowed_as_diagnostic") is True
            if isinstance(validation, dict) and isinstance(manifest_contract, dict)
            else False
        )
        sparse_counts = {"blank_wls_row", "sparse_omitted_epoch"}
        if gap_diagnostic_allowed:
            sparse_counts.add("timestamp_gap")
        unexpected_classifications = (
            {
                str(key): int(value)
                for key, value in (classification_counts or {}).items()
                if str(key) not in sparse_counts and int(value) != 0
            }
            if isinstance(classification_counts, dict)
            else {"classification_counts": 1}
        )
        sparse_classification_failure = (
            not sparse_allowed
            or not manifest_sparse_allowed
            or not isinstance(classification_counts, dict)
            or any(
                int(classification_counts.get(key, 0)) < 0
                for key in sparse_counts
            )
        )
        if (
            manifest.get("truth_free") is not True
            or manifest.get("truth_used") is not False
            or not isinstance(validation, dict)
            or not isinstance(populations, dict)
            or validation.get("all_epoch_rows_consistent") is not True
            or validation.get("all_selected_rows_finite") is not True
            or not gap_count_valid
            or (gap_count > 0 and not gap_diagnostic_allowed)
            or unexpected_classifications
            or sparse_classification_failure
        ):
            raise HoldoutEnsembleError(f"WLS integrity failed closed for {phone}")
        summaries[phone] = {
            "manifest": _artifact(manifest_path),
            "summary": _artifact(summary_path),
            "classification_counts": classification_counts,
            "allow_missing_wls_epochs": sparse_allowed,
            "blank_wls_row_count": populations.get("blank_wls_row_count"),
            "sparse_omitted_epoch_count": populations.get("sparse_omitted_epoch_count"),
            "sparse_omitted_timestamps_ms": populations.get("sparse_omitted_timestamps_ms", []),
            "sparse_omitted_epoch_records": populations.get("sparse_omitted_epoch_records", []),
            "selected_sparse_omitted_epoch_count": populations.get(
                "selected_sparse_omitted_epoch_count"
            ),
            "timestamp_gap_count_gt_1500ms": gap_count,
            "max_timestamp_gap_s": populations["max_timestamp_gap_s"],
            "timestamp_gap_allowed_as_diagnostic": gap_diagnostic_allowed,
            "timestamp_gap_classification_count": int(
                classification_counts.get("timestamp_gap", 0)
            ),
            "selected_epochs": populations["selected_epochs"],
            "all_epoch_rows_consistent": validation["all_epoch_rows_consistent"],
            "all_selected_rows_finite": validation["all_selected_rows_finite"],
        }
    return summaries


def _score_and_delta(
    baseline: dict[str, Any], candidate: dict[str, Any]
) -> dict[str, Any]:
    gate = ensemble._gate(candidate, baseline, SIGNOFF_THRESHOLDS)
    return {
        "baseline": baseline,
        "candidate": candidate,
        "gate": gate,
    }


def _route_manifest(
    route_root: Path,
    holdout: dict[str, Any],
    materialized_devices: dict[str, Any],
    wls_artifacts: dict[str, Any],
    method_artifacts: dict[str, Any],
) -> Path:
    payload = {
        "schema_version": RUN_MANIFEST_SCHEMA_VERSION,
        "route": HOLDOUT_ROUTE,
        "truth_free": True,
        "truth_used": False,
        "phones": sorted(materialized_devices),
        "materialized_device_gnss": materialized_devices,
        "wls": wls_artifacts,
        "methods": method_artifacts,
        "required_truth_members": [record["ground_truth"] for record in holdout["records"]],
        "alignment_contract": {
            "unix_time_millis": "exact device timestamp keys with nearest cross-phone match",
            "tolerance_ms": ALIGNMENT_TOLERANCE_MS,
            "minimum_aligned_phone_count": MINIMUM_ALIGNED_PHONE_COUNT,
            "fallback": "single target-phone WLS row when no peer aligns",
            "target_timestamp_coverage": "every target device timestamp must have at least one source phone",
            "extrapolation": "forbidden",
        },
    }
    path = route_root / "route_truth_free_manifest.json"
    _atomic_json(path, payload)
    return path


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get(
            "GNSS_CLI_NAME", "gnss smartphone-wls-multi-phone-ensemble-holdout-eval"
        )
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION)
    parser.add_argument("--freeze-record", type=Path, default=DEFAULT_FREEZE)
    parser.add_argument("--freeze-manifest", type=Path, default=DEFAULT_FREEZE_MANIFEST)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--materialized-input-dir",
        type=Path,
        help="Reuse a previously sealed device-only materialization; no archive extraction is repeated.",
    )
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    started = time.perf_counter()
    rss_before = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    truth_open_count = 0
    holdout_materialized = False
    try:
        if args.output_dir.exists() and any(args.output_dir.iterdir()):
            raise HoldoutEnsembleError(f"refusing to reuse non-empty output: {args.output_dir}")
        selection, _inventory, archive_hash, inventory_hash = ensemble._verify_selected_record(
            args.archive, args.inventory, args.selection_record
        )
        freeze, freeze_hash, freeze_manifest_hash = _verify_freeze(
            args.freeze_record,
            args.freeze_manifest,
            args.selection_record,
            selection,
            archive_hash,
            inventory_hash,
        )
        _verify_holdout_central_metadata(args.archive, freeze)
        args.output_dir.mkdir(parents=True, exist_ok=True)
        holdout = selection["next_holdout"]
        route_root = args.output_dir / ensemble._safe_id(HOLDOUT_ROUTE)
        inputs_root = route_root / "inputs"
        materialized_devices: dict[str, Any] = {}
        device_paths: dict[str, Path] = {}
        for record in holdout["records"]:
            dataset_id = str(record["dataset_id"])
            phone = dataset_id.split("/", 1)[1]
            if args.materialized_input_dir is not None:
                source = args.materialized_input_dir / phone / "device_gnss.csv"
                if not source.is_file() or source.stat().st_size != int(record["device_gnss"]["file_size"]):
                    raise HoldoutEnsembleError(f"reused device materialization is invalid: {source}")
                output = source
                materialized_devices[phone] = {
                    "member": ensemble._member_names(dataset_id)["device_gnss"],
                    "metadata": record["device_gnss"],
                    **_artifact(source),
                    "reused_existing_materialization": True,
                }
            else:
                output = inputs_root / phone / "device_gnss.csv"
                materialized_devices[phone] = ensemble._materialize_member(
                    args.archive,
                    ensemble._member_names(dataset_id)["device_gnss"],
                    output,
                    record["device_gnss"],
                )
            device_paths[phone] = output
        if args.materialized_input_dir is not None:
            materialized_devices = _verify_reused_device_materialization(
                args.materialized_input_dir, freeze, holdout
            )
        holdout_materialized = True

        positions_by_phone: dict[str, list[smoother.PositionRow]] = {}
        timestamp_keys_by_phone: dict[str, list[int]] = {}
        wls_artifacts: dict[str, Any] = {}
        for record in holdout["records"]:
            dataset_id = str(record["dataset_id"])
            phone = dataset_id.split("/", 1)[1]
            wls_root = route_root / "wls" / phone
            payload = wls.extract_to_directory(
                device_paths[phone],
                wls_root,
                skip_epochs=ensemble.SKIP_EPOCHS,
                role="holdout",
                dataset_id=dataset_id,
                sealed_holdout_eval_freeze=args.freeze_record,
                sealed_holdout_eval_freeze_manifest=args.freeze_manifest,
                truth_free=True,
                allow_missing_wls_epochs=True,
            )
            positions = smoother._read_positions(wls_root / "wls.pos", ensemble.LEAP_SECONDS)
            if not positions:
                raise HoldoutEnsembleError(f"WLS produced no positions for {dataset_id}")
            positions_by_phone[phone] = positions
            summary = _load_json(wls_root / "wls_summary.json", f"WLS summary for {phone}")
            selected_keys = summary.get("selection", {}).get(
                "selected_device_epoch_timestamps_ms"
            )
            if not isinstance(selected_keys, list) or not selected_keys:
                raise HoldoutEnsembleError(
                    f"WLS summary has no selected device timestamps for {dataset_id}"
                )
            try:
                timestamp_keys_by_phone[phone] = [int(timestamp) for timestamp in selected_keys]
            except (TypeError, ValueError) as exc:
                raise HoldoutEnsembleError(
                    f"WLS summary selected timestamps are invalid for {dataset_id}"
                ) from exc
            wls_artifacts[phone] = {
                "payload": payload,
                "position": _artifact(wls_root / "wls.pos"),
                "manifest": _artifact(wls_root / "wls_manifest.json"),
                "summary": _artifact(wls_root / "wls_summary.json"),
            }
        wls_integrity = _verify_wls_integrity(wls_artifacts)

        method_positions: dict[str, dict[str, list[smoother.PositionRow]]] = {}
        method_artifacts: dict[str, Any] = {}
        for method_id in (BASELINE_METHOD, SELECTED_METHOD):
            output_positions, artifacts = ensemble._write_method_outputs(
                route_root,
                holdout["records"],
                device_paths,
                positions_by_phone,
                method_id,
                {},
                timestamp_keys_by_phone=timestamp_keys_by_phone,
                alignment_tolerance_ms=ALIGNMENT_TOLERANCE_MS,
            )
            method_positions[method_id] = output_positions
            method_artifacts[method_id] = artifacts
        selected_alignment = method_artifacts[SELECTED_METHOD]["alignment"]
        if (
            selected_alignment.get("unresolved_target_epoch_count") != 0
            or selected_alignment.get("all_target_timestamps_covered_by_source") is not True
            or selected_alignment.get("extrapolation_used") is not False
            or selected_alignment.get("covered_target_epoch_count")
            != selected_alignment.get("target_epoch_count")
        ):
            raise HoldoutEnsembleError(
                "selected ensemble alignment is not fully covered without extrapolation"
            )
        route_manifest_path = _route_manifest(
            route_root,
            holdout,
            materialized_devices,
            wls_artifacts,
            method_artifacts,
        )
        truth_free_manifest = {
            "schema_version": SCHEMA_VERSION + ".truth-free",
            "truth_free": True,
            "truth_used": False,
            "freeze_record": {"path": str(args.freeze_record), "sha256": freeze_hash},
            "freeze_manifest": {
                "path": str(args.freeze_manifest),
                "sha256": freeze_manifest_hash,
            },
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
            "route_manifest": _artifact(route_manifest_path),
            "route": HOLDOUT_ROUTE,
            "phones": sorted(device_paths),
            "selected_method": SELECTED_METHOD,
            "baseline_method": BASELINE_METHOD,
            "selected_device_epoch_timestamps_ms": timestamp_keys_by_phone,
            "materialized_device_count": len(device_paths),
            "materialized_truth_count": 0,
            "wls_integrity": wls_integrity,
            "methods": method_artifacts,
            "alignment_contract": {
                "target_timestamp_coverage_required": True,
                "unresolved_target_epoch_policy": "fail-closed",
                "alignment_tolerance_ms": ALIGNMENT_TOLERANCE_MS,
                "extrapolation_policy": "forbidden",
            },
            "all_lane_artifacts_hash_fixed_before_truth": True,
            "next_holdout_materialized": True,
            "next_holdout_truth_opened": False,
            "post_holdout_tuning": False,
        }
        truth_free_path = args.output_dir / "truth_free_manifest.json"
        _atomic_json(truth_free_path, truth_free_manifest)
        truth_free_artifact = _artifact(truth_free_path)

        validation_truths: dict[str, dict[int, tuple[float, float, float]]] = {}
        truth_artifacts: dict[str, Any] = {}
        for record in holdout["records"]:
            dataset_id = str(record["dataset_id"])
            phone = dataset_id.split("/", 1)[1]
            truth_path = inputs_root / phone / "ground_truth.csv"
            truth_artifacts[phone] = ensemble._materialize_member(
                args.archive,
                ensemble._member_names(dataset_id)["ground_truth"],
                truth_path,
                record["ground_truth"],
            )
            validation_truths[phone] = ensemble._truth_read(truth_path)
            truth_open_count += 1

        baseline_score = ensemble._score_methods(
            {"method_positions": {BASELINE_METHOD: method_positions[BASELINE_METHOD]}},
            validation_truths,
        )[BASELINE_METHOD]
        selected_score = ensemble._score_methods(
            {"method_positions": {SELECTED_METHOD: method_positions[SELECTED_METHOD]}},
            validation_truths,
        )[SELECTED_METHOD]
        comparison = _score_and_delta(baseline_score, selected_score)
        per_phone_failures: list[str] = []
        for phone, score in selected_score["phone_metrics"].items():
            if score["availability_ratio"] < SIGNOFF_THRESHOLDS["availability_min"]:
                per_phone_failures.append(f"{phone}:availability")
            if score["truth_coverage_ratio"] < SIGNOFF_THRESHOLDS["truth_coverage_min"]:
                per_phone_failures.append(f"{phone}:coverage")
        if per_phone_failures:
            comparison["gate"]["failures"].extend(per_phone_failures)
            comparison["gate"]["passed"] = False
        comparison["gate"]["per_phone_signoff_failures"] = per_phone_failures
        promotion = (
            "promote-development-only-full-submission-integration-candidate"
            if comparison["gate"]["passed"]
            else "no-go-holdout-gate"
        )
        alignment = method_artifacts[SELECTED_METHOD]["alignment"]
        rss_after = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
        report = {
            "schema_version": SCHEMA_VERSION,
            "status": "sealed-completed-one-shot",
            "freeze": {
                "record": {"path": str(args.freeze_record), "sha256": freeze_hash},
                "manifest": {
                    "path": str(args.freeze_manifest),
                    "sha256": freeze_manifest_hash,
                },
            },
            "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
            "archive": {"path": str(args.archive), "sha256": archive_hash},
            "inventory": {"path": str(args.inventory), "sha256": inventory_hash},
            "route": HOLDOUT_ROUTE,
            "phones": sorted(device_paths),
            "wls_integrity": wls_integrity,
            "truth_free": {
                "manifest": truth_free_artifact,
                "truth_free_before_truth": True,
                "all_lane_artifacts_hash_fixed_before_truth": True,
                "device_materialized_count": len(device_paths),
                "truth_materialized_before_manifest": 0,
                "selected_method": SELECTED_METHOD,
                "alignment": alignment,
            },
            "truth_evaluation": {
                "truth_member_open_count": truth_open_count,
                "evaluation_pass_count": 1,
                "artifacts": truth_artifacts,
                "rows_by_phone": {phone: len(rows) for phone, rows in validation_truths.items()},
            },
            "metrics": comparison,
            "phone_level_metrics": {
                "baseline_single_phone_wls": baseline_score["phone_metrics"],
                "selected_multi_phone_median": selected_score["phone_metrics"],
            },
            "alignment_and_fallback": {
                "selected_method": SELECTED_METHOD,
                "phone_count_histogram": alignment["phone_count_histogram"],
                "missing_phone_observations": alignment["missing_phone_observations"],
                "fallback_one_phone_observations": alignment["phone_count_histogram"].get("1", 0),
                "minimum_aligned_phone_count": MINIMUM_ALIGNED_PHONE_COUNT,
                "spread_ecef_m": alignment["spread_ecef_m"],
                "max_abs_alignment_offset_ms": alignment["max_abs_alignment_offset_ms"],
            },
            "promotion": {
                "decision": promotion,
                "development_only_full_submission_integration_candidate": comparison["gate"]["passed"],
                "production_rtk_spp_default_changed": False,
                "post_holdout_tuning": False,
                "rerun_permitted": False,
            },
            "holdout_access": {
                "materialized": holdout_materialized,
                "truth_open_count": truth_open_count,
                "truth_materialized_after_truth_free_seal": True,
                "no_other_holdout_access": True,
            },
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "process_peak_rss_kb": max(int(rss_before), int(rss_after)),
                "process_rss_unit": "kilobytes from getrusage ru_maxrss",
            },
        }
        report_path = args.output_dir / "wls_multi_phone_ensemble_holdout_report.json"
        _atomic_json(report_path, report)
        run_manifest = {
            "schema_version": RUN_MANIFEST_SCHEMA_VERSION,
            "sealed": True,
            "report": _artifact(report_path),
            "truth_free_manifest": truth_free_artifact,
            "selected_method": SELECTED_METHOD,
            "baseline_method": BASELINE_METHOD,
            "promotion_decision": promotion,
            "truth_open_count": truth_open_count,
            "evaluation_pass_count": 1,
            "next_holdout_materialized": holdout_materialized,
            "truth_free_before_truth": True,
            "post_holdout_tuning": False,
        }
        _atomic_json(args.output_dir / "wls_multi_phone_ensemble_holdout_manifest.json", run_manifest)
        print(f"Smartphone WLS multi-phone holdout evaluation complete: {report_path}")
        print(f"Selected method: {SELECTED_METHOD}; promotion: {promotion}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        ensemble.EnsembleError,
        HoldoutEnsembleError,
        wls.WlsPositionError,
        smoother.SmootherError,
    ) as exc:
        print(f"Smartphone WLS multi-phone holdout failed: {exc}", file=sys.stderr)
        if args.output_dir.exists():
            try:
                _atomic_json(
                    args.output_dir / "wls_multi_phone_ensemble_holdout_failure.json",
                    {
                        "schema_version": SCHEMA_VERSION,
                        "status": "sealed-failed",
                        "error": str(exc),
                        "holdout_materialized": holdout_materialized,
                        "truth_open_count": truth_open_count,
                        "evaluation_pass_count": 0,
                        "post_holdout_tuning": False,
                        "rerun_permitted": False,
                    },
                )
            except OSError:
                pass
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
