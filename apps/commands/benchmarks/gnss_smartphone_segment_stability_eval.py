#!/usr/bin/env python3
"""Evaluate a truth-free segment stability gate for smartphone trajectories.

The route roles and stability thresholds are frozen in a record before the
new validation member is opened.  The filter, segment decisions, and raw
fallback artifacts are generated without truth; labels are parsed only for
the final score gate.  The designated holdout is never materialized.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_reacquisition_conservative_eval as conservative  # noqa: E402
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_segment_stability as stability  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-segment-stability-evaluation.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_GENERALIZATION_ROOT = ROOT / "output" / "smartphone-r5" / "generalization-v6"
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "segment-stability-v1"
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_segment_stability_candidates.json"
)

TRAIN_IDS = previous.TRAIN_IDS
MAIN_ID = previous.MAIN_ID
HOLDOUT_ID = previous.HOLDOUT_ID
NEW_VALIDATION_ID = "2021-07-14-20-50-us-ca-mtv-e/pixel4"
PREVIOUSLY_USED_IDS = (
    *previous.TRAIN_IDS,
    *previous.VALIDATION_IDS,
    previous.MAIN_ID,
    previous.HOLDOUT_ID,
    conservative.NEW_VALIDATION_ID,
)
TRAIN_EVAL_IDS = (*TRAIN_IDS, MAIN_ID)
CANDIDATES = (
    {
        "id": "segment_r15_d15",
        "max_consecutive_rejects": 15,
        "max_prediction_duration_s": 15.0,
        "reject_fraction_max": None,
    },
    {
        "id": "segment_r20_d20",
        "max_consecutive_rejects": 20,
        "max_prediction_duration_s": 20.0,
        "reject_fraction_max": None,
    },
    {
        "id": "segment_r30_d30",
        "max_consecutive_rejects": 30,
        "max_prediction_duration_s": 30.0,
        "reject_fraction_max": None,
    },
)
BASELINE_CONFIG = dict(previous.BASELINE_CONFIG)
DIAGNOSTIC_KEYS = previous._DIAGNOSTIC_KEYS


class SegmentStabilityEvaluationError(ValueError):
    """Raised when the frozen stability evaluation contract fails."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise SegmentStabilityEvaluationError(f"missing file: {path}")
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _load_selection_record(path: Path) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SegmentStabilityEvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict):
        raise SegmentStabilityEvaluationError("selection record must be an object")
    if payload.get("schema_version") != "smartphone-r5-segment-stability-selection.v1":
        raise SegmentStabilityEvaluationError("selection record schema is not segment stability v1")
    if payload.get("status") != "selection-frozen-before-evaluation":
        raise SegmentStabilityEvaluationError("selection record is not frozen before evaluation")
    roles = payload.get("fixed_roles")
    if not isinstance(roles, dict):
        raise SegmentStabilityEvaluationError("selection record has no fixed roles")
    if tuple(roles.get("candidate_train", ())) != TRAIN_EVAL_IDS:
        raise SegmentStabilityEvaluationError("selection record train roles differ from the contract")
    if tuple(roles.get("new_validation", ())) != (NEW_VALIDATION_ID,):
        raise SegmentStabilityEvaluationError("selection record new validation differs from the contract")
    selected = payload.get("new_validation_selection")
    if not isinstance(selected, dict) or selected.get("dataset_id") != NEW_VALIDATION_ID:
        raise SegmentStabilityEvaluationError("selection record does not freeze the expected validation")
    if payload.get("candidate_set") != list(CANDIDATES):
        raise SegmentStabilityEvaluationError("selection record candidate set differs from the contract")
    sealed = payload.get("sealed_data_policy")
    if not isinstance(sealed, dict) or sealed.get("designated_holdout_id") != HOLDOUT_ID:
        raise SegmentStabilityEvaluationError("selection record holdout differs from the contract")
    return payload


def _select_new_validation(
    archive_path: Path,
    inventory: dict[str, Any],
    profile: dict[str, Any],
    record: dict[str, Any],
    generalization_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    development_id = str(dict(profile["datasets"]["development"])["id"])
    holdout_id = str(dict(profile["datasets"]["holdout"])["id"])
    if holdout_id != HOLDOUT_ID:
        raise SegmentStabilityEvaluationError("profile holdout differs from sealed contract")
    if NEW_VALIDATION_ID in PREVIOUSLY_USED_IDS or NEW_VALIDATION_ID in (development_id, holdout_id):
        raise SegmentStabilityEvaluationError("new validation collides with used or sealed IDs")
    route, phone = NEW_VALIDATION_ID.split("/", 1)
    prior_materialized = generalization_root / "routes" / route / phone
    if prior_materialized.exists():
        raise SegmentStabilityEvaluationError(
            f"new validation was already materialized before this experiment: {prior_materialized}"
        )
    records = {row["dataset_id"]: row for row in inventory["train"]["records"]}
    candidate = records.get(NEW_VALIDATION_ID)
    if candidate is None:
        raise SegmentStabilityEvaluationError("new validation is absent from central inventory")
    if candidate["route"] in {value.split("/", 1)[0] for value in PREVIOUSLY_USED_IDS}:
        raise SegmentStabilityEvaluationError("new validation reuses a prior route")
    if candidate["phone"] in {value.split("/", 1)[1] for value in PREVIOUSLY_USED_IDS}:
        raise SegmentStabilityEvaluationError("new validation reuses a prior phone model")
    screen = conservative._signal_only_screen(archive_path, inventory, NEW_VALIDATION_ID)
    expected = dict(record["new_validation_selection"]).get("screen_result")
    if not isinstance(expected, dict):
        raise SegmentStabilityEvaluationError("selection record has no signal-only screen result")
    for key in (
        "valid_galileo_e1_rows",
        "galileo_e1_epochs",
        "galileo_e1_svid_count",
        "unique_broadcast_nav_present",
    ):
        if screen[key] != expected.get(key):
            raise SegmentStabilityEvaluationError(
                f"signal-only screen changed for {NEW_VALIDATION_ID}: {key}={screen[key]!r} expected={expected.get(key)!r}"
            )
    return candidate, screen


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _write_stability_outputs(
    result: smoother.SmoothingResult,
    stability_report: dict[str, Any],
    output_dir: Path,
    position_path: Path,
    device_path: Path,
) -> Path:
    baseline_manifest = smoother.write_outputs(
        result,
        smoother.SmootherConfig(**BASELINE_CONFIG),
        output_dir,
        position_path=position_path,
        device_path=device_path,
        skip_epochs=1,
        leap_seconds=18,
    )
    report_path = output_dir / "segment_stability.json"
    report = {
        **stability_report,
        "raw_position": {"path": str(position_path), "sha256": _sha256(position_path)},
        "truth_used": False,
    }
    _atomic_json(report_path, report)
    manifest_path = output_dir / "smoother_manifest.json"
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SegmentStabilityEvaluationError("smoother manifest was not valid JSON") from exc
    manifest["segment_stability"] = {
        "path": str(report_path),
        "sha256": _sha256(report_path),
        "raw_position": report["raw_position"],
        "thresholds": report["thresholds"],
        "population": report["population"],
    }
    manifest.setdefault("artifacts", {})["segment_stability"] = _artifact(report_path)
    _atomic_json(manifest_path, manifest)
    if baseline_manifest.get("artifacts", {}).get("smoothed_pos") is None:
        raise SegmentStabilityEvaluationError("smoother did not publish POS artifact")
    return manifest_path


def _filter_route(
    dataset_id: str,
    position_path: Path,
    device_path: Path,
    truth_path: Path,
    output_dir: Path,
    *,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    for path, label in ((position_path, "position"), (device_path, "device GNSS")):
        if not path.is_file():
            raise SegmentStabilityEvaluationError(f"missing {label} for {dataset_id}: {path}")
    positions = smoother._read_positions(position_path, 18)
    epochs = smoother._read_device_epochs(device_path, 1)
    baseline_config = smoother.SmootherConfig(**BASELINE_CONFIG)
    baseline = smoother.smooth_positions(positions, epochs, baseline_config)
    route_output = output_dir / "routes" / _safe_id(dataset_id)
    baseline_dir = route_output / "existing-smoother"
    baseline_manifest = smoother.write_outputs(
        baseline,
        baseline_config,
        baseline_dir,
        position_path=position_path,
        device_path=device_path,
        skip_epochs=1,
        leap_seconds=18,
    )
    candidate_results: dict[str, smoother.SmoothingResult] = {}
    candidate_reports: dict[str, dict[str, Any]] = {}
    candidate_dirs: dict[str, Path] = {"existing-smoother": baseline_dir}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        result, stability_report = stability.apply_segment_stability(
            baseline,
            positions,
            epochs,
            max_consecutive_rejects=int(candidate["max_consecutive_rejects"]),
            max_prediction_duration_s=float(candidate["max_prediction_duration_s"]),
            reject_fraction_max=candidate["reject_fraction_max"],
            measurement_floor_m=float(BASELINE_CONFIG["measurement_floor_m"]),
            leap_seconds=18,
        )
        candidate_dir = route_output / candidate_id
        candidate_dirs[candidate_id] = candidate_dir
        manifest_path = _write_stability_outputs(
            result,
            stability_report,
            candidate_dir,
            position_path,
            device_path,
        )
        candidate_results[candidate_id] = result
        candidate_reports[candidate_id] = {
            **stability_report,
            "manifest": {"path": str(manifest_path), "sha256": _sha256(manifest_path)},
        }
    return {
        "dataset_id": dataset_id,
        "position": position_path,
        "device": device_path,
        "truth": truth_path,
        "positions": positions,
        "epochs": epochs,
        "baseline": baseline,
        "baseline_manifest": {
            "path": str(baseline_dir / "smoother_manifest.json"),
            "sha256": _sha256(baseline_dir / "smoother_manifest.json"),
        },
        "candidates": candidate_results,
        "candidate_reports": candidate_reports,
        "output_dirs": candidate_dirs,
        "input_hashes": {
            "position": _sha256(position_path),
            "device": _sha256(device_path),
        },
        "extra": extra or {},
    }


def _score_group(
    route_results: dict[str, dict[str, Any]],
    route_ids: tuple[str, ...],
    *,
    candidate_id: str | None = None,
) -> dict[str, dict[str, Any]]:
    scored: dict[str, dict[str, Any]] = {}
    for dataset_id in route_ids:
        route = route_results[dataset_id]
        truth_path = Path(route["truth"])
        truth = smoother_eval._read_truth(truth_path)
        route["input_hashes"]["truth"] = _sha256(truth_path)
        if candidate_id is None:
            rows = previous._raw_rows(route["positions"])
        elif candidate_id == "existing-smoother":
            rows = route["baseline"].rows
        else:
            rows = route["candidates"][candidate_id].rows
        scored[dataset_id] = previous._score(
            rows, route["positions"], route["epochs"], truth
        )
    return scored


def _compare(candidate: dict[str, Any], reference: dict[str, Any]) -> dict[str, Any]:
    return previous._compare_metrics(candidate, {"reference": reference})


def _candidate_report(
    route_results: dict[str, dict[str, Any]],
    train_scores: dict[str, dict[str, dict[str, Any]]],
    train_raw: dict[str, dict[str, Any]],
    train_existing: dict[str, dict[str, Any]],
    validation_raw: dict[str, dict[str, Any]],
    validation_existing: dict[str, dict[str, Any]],
    validation_scores: dict[str, dict[str, Any]],
    main_scores: dict[str, dict[str, Any]],
    main_existing: dict[str, dict[str, Any]],
    main_byte_identity: dict[str, bool],
) -> tuple[dict[str, Any], str | None]:
    existing_train = previous._aggregate(
        [train_scores["existing-smoother"][dataset_id] for dataset_id in TRAIN_EVAL_IDS]
    )
    values: dict[str, Any] = {}
    stable_route_ids: dict[str, list[str]] = {}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        raw_failures: dict[str, list[str]] = {}
        existing_failures: dict[str, list[str]] = {}
        all_stable = []
        for dataset_id in TRAIN_EVAL_IDS:
            route = route_results[dataset_id]
            report = route["candidate_reports"][candidate_id]
            if report["population"]["unstable_segment_count"] == 0:
                all_stable.append(dataset_id)
            raw_failures[dataset_id] = _compare(
                train_scores[candidate_id][dataset_id], train_raw[dataset_id]
            )["failures_by_reference"]["reference"]
            if report["population"]["unstable_segment_count"] == 0:
                existing_failures[dataset_id] = _compare(
                    train_scores[candidate_id][dataset_id], train_existing[dataset_id]
                )["failures_by_reference"]["reference"]
        stable_route_ids[candidate_id] = all_stable
        train_aggregate = previous._aggregate(
            [train_scores[candidate_id][dataset_id] for dataset_id in TRAIN_EVAL_IDS]
        )
        values[candidate_id] = {
            "candidate": candidate,
            "train_aggregate": train_aggregate,
            "train_raw_failures_by_route": raw_failures,
            "train_stable_route_existing_failures_by_route": existing_failures,
            "train_raw_non_regression_passed": not any(raw_failures.values()),
            "train_stable_route_non_regression_passed": not any(existing_failures.values()),
            "main_pos_byte_identical": main_byte_identity[candidate_id],
            "main_fallback_epochs": route_results[MAIN_ID]["candidate_reports"][candidate_id]["population"]["fallback_epochs"],
            "new_validation": validation_scores[candidate_id][NEW_VALIDATION_ID],
        }
    eligible = [
        value
        for value in values.values()
        if value["train_raw_non_regression_passed"]
        and value["train_stable_route_non_regression_passed"]
        and value["main_pos_byte_identical"]
        and value["main_fallback_epochs"] == 0
    ]
    selected = min(
        eligible,
        key=lambda value: (
            float(value["train_aggregate"]["mean_horizontal_wgs84_p95_m"]),
            float(value["train_aggregate"]["mean_horizontal_wgs84_p50_m"]),
            float(value["train_aggregate"]["mean_vertical_p95_abs_m"]),
            float(value["train_aggregate"]["mean_kaggle_diagnostic_m"]),
            str(value["candidate"]["id"]),
        ),
    ) if eligible else None
    selected_id = str(selected["candidate"]["id"]) if selected else None
    validation_gate = None
    stable_validation_gate = None
    main_gate = None
    if selected_id is not None:
        validation_gate = _compare(
            validation_scores[selected_id][NEW_VALIDATION_ID],
            validation_raw[NEW_VALIDATION_ID],
        )
        validation_route = route_results[NEW_VALIDATION_ID]
        if validation_route["candidate_reports"][selected_id]["population"]["unstable_segment_count"] == 0:
            stable_validation_gate = _compare(
                validation_scores[selected_id][NEW_VALIDATION_ID],
                validation_existing[NEW_VALIDATION_ID],
            )
        else:
            stable_validation_gate = {
                "non_regression_passed": True,
                "failures_by_reference": {},
                "not_applicable": True,
            }
        main_gate = _compare(
            main_scores[selected_id][MAIN_ID],
            main_existing[MAIN_ID],
        )
        main_gate["pos_byte_identical"] = main_byte_identity[selected_id]
        main_gate["fallback_epochs_zero"] = (
            route_results[MAIN_ID]["candidate_reports"][selected_id]["population"]["fallback_epochs"] == 0
        )
        main_gate["non_regression_passed"] = bool(
            main_gate["non_regression_passed"]
            and main_gate["pos_byte_identical"]
            and main_gate["fallback_epochs_zero"]
        )
    promotion = "no-go-no-eligible-train-candidate"
    if selected_id is not None:
        if not validation_gate["non_regression_passed"]:
            promotion = "no-go-new-validation-raw-regression"
        elif not stable_validation_gate["non_regression_passed"]:
            promotion = "no-go-new-validation-stable-segment-regression"
        elif not main_gate["non_regression_passed"]:
            promotion = "no-go-development-main-regression-or-contract"
        else:
            promotion = "promote-development-only-recommended-pipeline"
    return (
        {
            "existing_train_aggregate": existing_train,
            "candidates": values,
            "stable_route_ids_by_candidate": stable_route_ids,
            "selected_candidate_id": selected_id,
            "new_validation_raw_gate": validation_gate,
            "new_validation_stable_segment_gate": stable_validation_gate,
            "development_main_gate": main_gate,
            "promotion_decision": promotion,
            "existing_smoother_retained": promotion != "promote-development-only-recommended-pipeline",
        },
        selected_id,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-segment-stability-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--generalization-root", type=Path, default=DEFAULT_GENERALIZATION_ROOT)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--main-position", type=Path, default=previous.DEFAULT_MAIN_POSITION)
    parser.add_argument("--main-device-gnss", type=Path, default=previous.DEFAULT_MAIN_DEVICE)
    parser.add_argument("--main-ground-truth", type=Path, default=previous.DEFAULT_MAIN_TRUTH)
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        if args.max_epochs == 0 or args.max_epochs < -1:
            raise SegmentStabilityEvaluationError("max-epochs must be -1 or a positive integer")
        record = _load_selection_record(args.selection_record)
        profile = previous._load_profile(args.profile)
        inventory = generalization.inventory_archive(args.archive)
        candidate, signal_screen = _select_new_validation(
            args.archive,
            inventory,
            profile,
            record,
            args.generalization_root,
        )
        archive_expected = str(dict(profile["archive"]).get("sha256", ""))
        if not archive_expected:
            raise SegmentStabilityEvaluationError("profile archive SHA-256 is empty")
        archive_sha = _sha256(args.archive)
        if archive_sha != archive_expected:
            raise SegmentStabilityEvaluationError("archive SHA-256 differs from frozen profile")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        inventory["selection"] = {
            "fixed_train_ids": list(TRAIN_EVAL_IDS),
            "fixed_new_validation_id": NEW_VALIDATION_ID,
            "previous_validation_not_reused": list(previous.VALIDATION_IDS) + [conservative.NEW_VALIDATION_ID],
            "development_excluded": MAIN_ID,
            "holdout_excluded": HOLDOUT_ID,
            "selection_content_read": False,
            "signal_only_screen": signal_screen,
        }
        inventory["archive"].update(
            {
                "sha256": archive_sha,
                "archive_hash_verified_before_member_extraction": True,
                "member_content_read": False,
                "member_sha256_computed": False,
            }
        )
        inventory_path = args.output_dir / "archive_inventory.json"
        _atomic_json(inventory_path, inventory)
        started = time.perf_counter()
        candidate_profile, materialized, profile_path = conservative._materialize_new_validation(
            args.archive,
            profile,
            candidate,
            archive_sha,
            args.output_dir,
        )
        generated = conservative._run_adapter_and_spp(
            candidate,
            candidate_profile,
            materialized,
            profile_path,
            args.output_dir,
            max_epochs=args.max_epochs,
        )
        route_results: dict[str, dict[str, Any]] = {}
        for dataset_id in TRAIN_IDS:
            inputs = previous._route_inputs(
                dataset_id,
                args.generalization_root,
                main_position=args.main_position,
                main_device=args.main_device_gnss,
                main_truth=args.main_ground_truth,
            )
            route_results[dataset_id] = _filter_route(
                dataset_id,
                Path(inputs["position"]),
                Path(inputs["device"]),
                Path(inputs["truth"]),
                args.output_dir,
            )
        route_results[MAIN_ID] = _filter_route(
            MAIN_ID,
            args.main_position,
            args.main_device_gnss,
            args.main_ground_truth,
            args.output_dir,
        )
        route_results[NEW_VALIDATION_ID] = _filter_route(
            NEW_VALIDATION_ID,
            Path(generated["position"]),
            Path(generated["device"]),
            Path(generated["truth"]),
            args.output_dir,
            extra={
                "truth_free_pipeline": True,
                "materialization": materialized,
                "candidate_profile": profile_path,
                "adapter_summary": generated["adapter_summary"],
                "spp_summary": generated["spp_summary"],
                "stages": generated["stages"],
            },
        )

        train_raw = _score_group(route_results, TRAIN_EVAL_IDS)
        train_existing = _score_group(route_results, TRAIN_EVAL_IDS, candidate_id="existing-smoother")
        train_scores: dict[str, dict[str, dict[str, Any]]] = {
            "raw_spp_hatch": train_raw,
            "existing-smoother": train_existing,
        }
        for candidate_spec in CANDIDATES:
            candidate_id = str(candidate_spec["id"])
            train_scores[candidate_id] = _score_group(
                route_results, TRAIN_EVAL_IDS, candidate_id=candidate_id
            )
        validation_raw = _score_group(route_results, (NEW_VALIDATION_ID,))
        validation_existing = _score_group(
            route_results, (NEW_VALIDATION_ID,), candidate_id="existing-smoother"
        )
        validation_scores = {
            str(candidate_spec["id"]): _score_group(
                route_results, (NEW_VALIDATION_ID,), candidate_id=str(candidate_spec["id"])
            )
            for candidate_spec in CANDIDATES
        }
        main_existing = _score_group(route_results, (MAIN_ID,), candidate_id="existing-smoother")
        main_scores = {
            str(candidate_spec["id"]): _score_group(
                route_results, (MAIN_ID,), candidate_id=str(candidate_spec["id"])
            )
            for candidate_spec in CANDIDATES
        }
        main_baseline_pos = route_results[MAIN_ID]["output_dirs"]["existing-smoother"] / "smoothed.pos"
        main_byte_identity = {
            str(candidate_spec["id"]): (
                (route_results[MAIN_ID]["output_dirs"][str(candidate_spec["id"])] / "smoothed.pos").read_bytes()
                == main_baseline_pos.read_bytes()
            )
            for candidate_spec in CANDIDATES
        }
        selection, selected_id = _candidate_report(
            route_results,
            train_scores,
            train_raw,
            train_existing,
            validation_raw,
            validation_existing,
            validation_scores,
            main_scores,
            main_existing,
            main_byte_identity,
        )
        routes: dict[str, Any] = {}
        for dataset_id, route in route_results.items():
            routes[dataset_id] = {
                "position": str(route["position"]),
                "device_gnss": str(route["device"]),
                "ground_truth": str(route["truth"]),
                "input_hashes": route["input_hashes"],
                "baseline_smoother": route["baseline_manifest"],
                "truth_free_pipeline": bool(route["extra"].get("truth_free_pipeline", False)),
                "stages": route["extra"].get("stages", []),
                "segment_decisions": {
                    candidate_id: {
                        "manifest": report["manifest"],
                        "thresholds": report["thresholds"],
                        "population": report["population"],
                        "segments": report["segments"],
                        "raw_position": {
                            "path": str(route["position"]),
                            "sha256": _sha256(route["position"]),
                        },
                    }
                    for candidate_id, report in route["candidate_reports"].items()
                },
            }
        report_path = args.output_dir / "segment_stability_report.json"
        manifest_path = args.output_dir / "segment_stability_manifest.json"
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-segment-stability-evaluation",
            "selection_record": str(args.selection_record),
            "archive": {
                "path": str(args.archive),
                "sha256": archive_sha,
                "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path)},
            },
            "roles": {
                "candidate_train": list(TRAIN_EVAL_IDS),
                "new_validation": [NEW_VALIDATION_ID],
                "previous_validation_not_reused": list(previous.VALIDATION_IDS) + [conservative.NEW_VALIDATION_ID],
                "development_main_regression": [MAIN_ID],
                "holdout_id": HOLDOUT_ID,
                "holdout_content_opened": False,
                "holdout_truth_opened": False,
                "holdout_materialized": False,
            },
            "evaluation_contract": {
                "truth_free_before_scoring": True,
                "signal_lane": "Galileo E1 with Hatch C1C 30 s",
                "baseline": BASELINE_CONFIG,
                "candidate_set": list(CANDIDATES),
                "segment_gate": {
                    "metrics": [
                        "max_consecutive_rejects",
                        "reject_fraction",
                        "max_normalized_innovation_sigma",
                        "max_prediction_duration_s",
                    ],
                    "unstable_policy": "all smoothed coordinates replaced with raw/Hatch POS",
                    "missing_device_key_policy": "explicit interpolation from nearest bracketing raw rows",
                    "reject_fraction_condition": "unused; recorded only",
                },
                "main_pos_byte_identity_required": True,
                "imu_adaptive_status": "No-Go retained; not evaluated or changed",
            },
            "new_validation_signal_only_screen": signal_screen,
            "materialization": {
                "manifest": str(materialized["manifest"]),
                "manifest_sha256": _sha256(materialized["manifest"]),
                "candidate_profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
                "truth_used_for_materialization": False,
            },
            "routes": routes,
            "scores": {
                "train": train_scores,
                "new_validation_raw_spp_hatch": validation_raw,
                "new_validation_existing_smoother": validation_existing,
                "new_validation_candidates": validation_scores,
                "development_main_existing_smoother": main_existing,
                "development_main_candidates": main_scores,
            },
            "selection": selection,
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "new_validation_pipeline_stages": generated["stages"],
                "smoother_wall_s": {
                    dataset_id: {
                        "existing": route["baseline"].elapsed_s,
                        **{
                            candidate_id: result.elapsed_s
                            for candidate_id, result in route["candidates"].items()
                        },
                    }
                    for dataset_id, route in route_results.items()
                },
            },
            "artifact_manifest": str(manifest_path),
        }
        _atomic_json(report_path, report)
        manifest = {
            "schema_version": "smartphone-r5-segment-stability-manifest.v1",
            "report": _artifact(report_path),
            "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
            "inventory": _artifact(inventory_path),
            "new_validation_materialization": {
                "path": str(materialized["manifest"]),
                "sha256": _sha256(materialized["manifest"]),
            },
            "route_artifacts": {
                dataset_id: {
                    "existing-smoother": route["baseline_manifest"],
                    **{
                        candidate_id: report["manifest"]
                        for candidate_id, report in route["candidate_reports"].items()
                    },
                }
                for dataset_id, route in route_results.items()
            },
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "holdout_materialized": False,
        }
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone segment stability evaluation complete: {report_path}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        generalization.GeneralizationError,
        conservative.ConservativeEvaluationError,
        previous.ReacquisitionError,
        stability.SegmentStabilityError,
        smoother.SmootherError,
        SegmentStabilityEvaluationError,
    ) as exc:
        print(f"Smartphone segment stability evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
