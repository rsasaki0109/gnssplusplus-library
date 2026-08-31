#!/usr/bin/env python3
"""Evaluate conservative truth-free smartphone smoother reacquisition bounds.

The candidate set and the new validation route are frozen in a pre-evaluation
record.  The archive is inspected through its central directory first; only
the selected device GNSS member is opened for the signal-only Galileo-E1
eligibility screen.  All adapter, SPP, and smoother artifacts are published
before any ground-truth row is parsed.
"""

from __future__ import annotations

import argparse
import csv
from datetime import datetime
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any
import zipfile

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_reacquisition_eval as previous  # noqa: E402
import gnss_smartphone_gnss_adapter as adapter  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-reacquisition-conservative-evaluation.v1"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_GENERALIZATION_ROOT = ROOT / "output" / "smartphone-r5" / "generalization-v6"
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "reacquisition-conservative-v1"
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_reacquisition_conservative_candidates.json"
)

TRAIN_IDS = previous.TRAIN_IDS
MAIN_ID = previous.MAIN_ID
HOLDOUT_ID = previous.HOLDOUT_ID
NEW_VALIDATION_ID = "2021-03-16-20-40-us-ca-mtv-b/pixel4xl"
PREVIOUSLY_SCORED_IDS = (
    *previous.TRAIN_IDS,
    *previous.VALIDATION_IDS,
    previous.MAIN_ID,
    HOLDOUT_ID,
)
CANDIDATES = (
    {
        "id": "reacq_r15_d15",
        "max_consecutive_rejects": 15,
        "max_prediction_duration_s": 15.0,
    },
    {
        "id": "reacq_r20_d20",
        "max_consecutive_rejects": 20,
        "max_prediction_duration_s": 20.0,
    },
    {
        "id": "reacq_r30_d30",
        "max_consecutive_rejects": 30,
        "max_prediction_duration_s": 30.0,
    },
)
BASELINE_CONFIG = dict(previous.BASELINE_CONFIG)
DIAGNOSTIC_KEYS = previous._DIAGNOSTIC_KEYS
REQUIRED_SCREEN_FIELDS = (
    "SignalType",
    "ConstellationType",
    "CarrierFrequencyHz",
    "CodeType",
    "Svid",
    "RawPseudorangeMeters",
    "utcTimeMillis",
)


class ConservativeEvaluationError(ValueError):
    """Raised when the frozen conservative evaluation contract fails."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ConservativeEvaluationError(f"missing file: {path}")
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
        raise ConservativeEvaluationError(f"invalid selection record: {path}") from exc
    if not isinstance(payload, dict):
        raise ConservativeEvaluationError("selection record must be an object")
    if payload.get("schema_version") != "smartphone-r5-reacquisition-conservative-selection.v1":
        raise ConservativeEvaluationError("selection record schema is not frozen conservative v1")
    if payload.get("status") != "selection-frozen-before-evaluation":
        raise ConservativeEvaluationError("selection record is not in pre-evaluation frozen state")
    roles = payload.get("fixed_roles")
    if not isinstance(roles, dict):
        raise ConservativeEvaluationError("selection record has no fixed roles")
    if tuple(roles.get("candidate_train", ())) != tuple(TRAIN_IDS):
        raise ConservativeEvaluationError("selection record train roles differ from the contract")
    if tuple(roles.get("new_validation", ())) != (NEW_VALIDATION_ID,):
        raise ConservativeEvaluationError("selection record new validation role differs from the contract")
    if tuple(roles.get("development_main_regression", ())) != (MAIN_ID,):
        raise ConservativeEvaluationError("selection record main role differs from the contract")
    selected = payload.get("new_validation_selection")
    if not isinstance(selected, dict) or selected.get("dataset_id") != NEW_VALIDATION_ID:
        raise ConservativeEvaluationError("selection record does not freeze the expected new validation")
    candidate_set = payload.get("candidate_set")
    if candidate_set != list(CANDIDATES):
        raise ConservativeEvaluationError("selection record candidate set differs from the contract")
    sealed = payload.get("sealed_data_policy")
    if not isinstance(sealed, dict) or sealed.get("designated_holdout_id") != HOLDOUT_ID:
        raise ConservativeEvaluationError("selection record holdout differs from the sealed profile")
    return payload


def _signal_only_screen(
    archive_path: Path,
    inventory: dict[str, Any],
    dataset_id: str,
) -> dict[str, Any]:
    """Read only the selected device GNSS member for E1 eligibility."""

    records = {
        row["dataset_id"]: row for row in inventory["train"]["records"]
    }
    row = records.get(dataset_id)
    if row is None:
        raise ConservativeEvaluationError(f"new validation is absent from archive inventory: {dataset_id}")
    if not row["required_files_complete"] or not row["broadcast_nav_present"]:
        raise ConservativeEvaluationError("new validation fails central-directory completeness")
    route, phone = dataset_id.split("/", 1)
    member = generalization._member_names(route, phone)["device_gnss"]
    valid_rows = 0
    invalid_e1_rows = 0
    e1_epochs: set[int] = set()
    e1_svids: set[int] = set()
    signal_rows: dict[str, int] = {}
    total_rows = 0
    last_timestamp: int | None = None
    try:
        with zipfile.ZipFile(archive_path) as archive:
            matches = [info for info in archive.infolist() if info.filename == member]
            if len(matches) != 1 or matches[0].is_dir():
                raise ConservativeEvaluationError(f"selected device member is not unique: {member}")
            with archive.open(matches[0], "r") as binary:
                text = io.TextIOWrapper(binary, encoding="utf-8-sig", newline="")
                reader = csv.DictReader(text)
                fields = set(reader.fieldnames or ())
                missing = sorted(set(REQUIRED_SCREEN_FIELDS) - fields)
                if missing:
                    raise ConservativeEvaluationError(
                        "selected device member lacks signal-screen fields: " + ", ".join(missing)
                    )
                for row_number, source_row in enumerate(reader, start=2):
                    total_rows += 1
                    signal = source_row.get("SignalType", "").strip()
                    signal_rows[signal] = signal_rows.get(signal, 0) + 1
                    if signal != adapter.GALILEO_E1_SIGNAL:
                        continue
                    try:
                        timestamp = int(float(source_row["utcTimeMillis"]))
                        constellation = source_row["ConstellationType"].strip()
                        frequency = float(source_row["CarrierFrequencyHz"])
                        code_type = source_row["CodeType"].strip()
                        svid = int(float(source_row["Svid"]))
                        pseudorange = float(source_row["RawPseudorangeMeters"])
                        valid = (
                            constellation == "6"
                            and math.isfinite(frequency)
                            and abs(frequency - adapter.GALILEO_E1_HZ) <= 1.0
                            and not code_type
                            and 1 <= svid <= 36
                            and math.isfinite(pseudorange)
                            and bool(source_row["RawPseudorangeMeters"].strip())
                        )
                    except (KeyError, TypeError, ValueError):
                        valid = False
                        timestamp = -1
                        svid = -1
                    if timestamp >= 0 and last_timestamp is not None and timestamp < last_timestamp:
                        raise ConservativeEvaluationError(
                            f"selected device member timestamp moved backwards at row {row_number}"
                        )
                    if timestamp >= 0:
                        last_timestamp = timestamp
                    if valid:
                        valid_rows += 1
                        e1_epochs.add(timestamp)
                        e1_svids.add(svid)
                    else:
                        invalid_e1_rows += 1
    except (OSError, zipfile.BadZipFile) as exc:
        raise ConservativeEvaluationError(f"failed signal-only screen: {member}") from exc
    result = {
        "dataset_id": dataset_id,
        "member": member,
        "total_device_rows": total_rows,
        "signal_rows": dict(sorted(signal_rows.items())),
        "valid_galileo_e1_rows": valid_rows,
        "galileo_e1_epochs": len(e1_epochs),
        "galileo_e1_svid_count": len(e1_svids),
        "invalid_galileo_e1_rows": invalid_e1_rows,
        "unique_broadcast_nav_present": bool(row["broadcast_nav_present"]),
        "truth_opened": False,
        "screen_rule": {
            "signal_type": adapter.GALILEO_E1_SIGNAL,
            "constellation_type": "6",
            "carrier_frequency_hz": adapter.GALILEO_E1_HZ,
            "carrier_frequency_tolerance_hz": 1.0,
            "code_type": "empty",
            "svid_range": [1, 36],
            "raw_pseudorange": "finite and present",
        },
    }
    if valid_rows < 1 or invalid_e1_rows:
        raise ConservativeEvaluationError(
            f"signal-only E1 eligibility failed for {dataset_id}: {result}"
        )
    return result


def _select_new_validation(
    archive_path: Path,
    inventory: dict[str, Any],
    profile: dict[str, Any],
    selection_record: dict[str, Any],
    generalization_root: Path,
) -> tuple[dict[str, Any], dict[str, Any]]:
    development_id = str(dict(profile["datasets"]["development"])["id"])
    holdout_id = str(dict(profile["datasets"]["holdout"])["id"])
    if holdout_id != HOLDOUT_ID:
        raise ConservativeEvaluationError("profile holdout does not match sealed contract")
    if NEW_VALIDATION_ID in PREVIOUSLY_SCORED_IDS or NEW_VALIDATION_ID in (development_id, holdout_id):
        raise ConservativeEvaluationError("new validation collides with a used or sealed route")
    route, phone = NEW_VALIDATION_ID.split("/", 1)
    materialized_path = generalization_root / "routes" / route / phone
    if materialized_path.exists():
        raise ConservativeEvaluationError(
            f"new validation was already materialized under the prior generalization root: {materialized_path}"
        )
    records = {row["dataset_id"]: row for row in inventory["train"]["records"]}
    candidate = records.get(NEW_VALIDATION_ID)
    if candidate is None:
        raise ConservativeEvaluationError("frozen new validation is absent from central inventory")
    if candidate["route"] in {item.split("/", 1)[0] for item in PREVIOUSLY_SCORED_IDS}:
        raise ConservativeEvaluationError("new validation reuses a previously used route")
    if candidate["phone"] in {item.split("/", 1)[1] for item in (*TRAIN_IDS, MAIN_ID)}:
        raise ConservativeEvaluationError("new validation reuses a fixed train/main phone model")
    screen = _signal_only_screen(archive_path, inventory, NEW_VALIDATION_ID)
    expected = dict(selection_record["new_validation_selection"]).get("screen_result")
    if not isinstance(expected, dict):
        raise ConservativeEvaluationError("selection record has no signal-only screen result")
    for key in (
        "valid_galileo_e1_rows",
        "galileo_e1_epochs",
        "galileo_e1_svid_count",
        "unique_broadcast_nav_present",
    ):
        if screen[key] != expected.get(key):
            raise ConservativeEvaluationError(
                f"signal-only screen changed for {NEW_VALIDATION_ID}: {key}={screen[key]!r} expected={expected.get(key)!r}"
            )
    return candidate, screen


def _materialize_new_validation(
    archive_path: Path,
    profile: dict[str, Any],
    candidate: dict[str, Any],
    archive_sha256: str,
    output_dir: Path,
) -> tuple[dict[str, Any], dict[str, Any], Path]:
    member_hashes = generalization._discover_member_hashes(
        archive_path,
        generalization._member_names(candidate["route"], candidate["phone"]),
    )
    candidate_profile = generalization._candidate_profile(
        profile, candidate, archive_sha256, member_hashes
    )
    materialized = generalization.materialize_candidate(
        archive_path,
        candidate_profile,
        candidate,
        archive_sha256,
        output_dir / "new-validation-materialized",
        member_hashes,
    )
    profile_path = materialized["root"] / "candidate_profile.json"
    _atomic_json(profile_path, candidate_profile)
    return candidate_profile, materialized, profile_path


def _run_adapter_and_spp(
    candidate: dict[str, Any],
    candidate_profile: dict[str, Any],
    materialized: dict[str, Any],
    profile_path: Path,
    output_dir: Path,
    *,
    max_epochs: int,
) -> dict[str, Any]:
    """Generate the selected route's RINEX/POS without opening truth."""

    route_root = output_dir / "routes" / _safe_id(candidate["dataset_id"]) / "truth-free-pipeline"
    adapter_dir = route_root / "adapter"
    spp_dir = route_root / "spp"
    route_root.mkdir(parents=True, exist_ok=True)
    adapter_dir.mkdir(parents=True, exist_ok=True)
    spp_dir.mkdir(parents=True, exist_ok=True)
    inputs = materialized["inputs"]
    device_path = inputs / "device_gnss.csv"
    nav_path = inputs / "brdc.nav"
    log_path = route_root / "pipeline.log"
    profile_archive = dict(candidate_profile["archive"])
    dataset = dict(candidate_profile["datasets"]["development"])
    cli = [sys.executable, str(ROOT / "apps" / "gnss.py")]
    adapter_command = [
        *cli,
        "smartphone-gnss-adapter",
        "--device-gnss",
        str(device_path),
        "--truth-free",
        "--output-dir",
        str(adapter_dir),
        "--dataset-id",
        candidate["dataset_id"],
        "--device-model",
        candidate["phone"],
        "--source-url",
        str(profile_archive.get("url", "local-profile")),
        "--source-terms",
        str(profile_archive.get("source_terms", "local evaluation only")),
        "--role",
        "development",
        "--skip-epochs",
        str(int(dataset.get("skip_epochs", 1))),
        "--experimental-galileo-e1",
        "--experimental-galileo-e1-hatch-window-s",
        "30",
        "--broadcast-nav",
        str(nav_path),
    ]
    if max_epochs > 0:
        adapter_command.extend(("--max-epochs", str(max_epochs)))
    stages = [generalization._run_stage("adapter_truth_free_galileo_e1_hatch30", adapter_command, log_path)]
    summary_path = adapter_dir / "summary.json"
    try:
        summary = json.loads(summary_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ConservativeEvaluationError("truth-free adapter did not publish a valid summary") from exc
    if summary.get("truth_free") is not True or summary.get("inputs", {}).get("ground_truth") is not None:
        raise ConservativeEvaluationError("adapter truth-free contract was not honored")
    raw_pos = spp_dir / "libgnsspp_spp.pos"
    raw_summary = spp_dir / "libgnsspp_spp_summary.json"
    stages.append(
        generalization._run_stage(
            "spp",
            generalization._spp_command(adapter_dir / "rover.obs", nav_path, raw_pos, raw_summary),
            log_path,
        )
    )
    canonical_pos = spp_dir / "canonical.pos"
    canonical_started = time.perf_counter()
    canonicalization = generalization._canonicalize_position_timestamps(
        raw_pos,
        device_path,
        canonical_pos,
        int(dataset.get("skip_epochs", 1)),
    )
    stages.append(
        {
            "stage": "position_timestamp_canonicalization",
            "wall_time_s": time.perf_counter() - canonical_started,
            "peak_rss_kb": None,
            "return_code": 0,
        }
    )
    return {
        "position": canonical_pos,
        "device": device_path,
        "truth": inputs / "ground_truth.csv",
        "nav": nav_path,
        "adapter_summary": summary_path,
        "spp_summary": raw_summary,
        "pipeline_root": route_root,
        "stages": stages,
        "canonicalization": canonicalization,
        "profile": profile_path,
    }


def _filter_route(
    dataset_id: str,
    position_path: Path,
    device_path: Path,
    truth_path: Path,
    output_dir: Path,
    *,
    stages: list[dict[str, Any]] | None = None,
    extra: dict[str, Any] | None = None,
) -> dict[str, Any]:
    for path, label in ((position_path, "position"), (device_path, "device GNSS")):
        if not path.is_file():
            raise ConservativeEvaluationError(f"missing {label} for {dataset_id}: {path}")
    positions = smoother._read_positions(position_path, 18)
    epochs = smoother._read_device_epochs(device_path, 1)
    baseline_config = smoother.SmootherConfig(**BASELINE_CONFIG)
    baseline_result = smoother.smooth_positions(positions, epochs, baseline_config)
    route_output = output_dir / "routes" / _safe_id(dataset_id)
    baseline_dir = route_output / "existing-smoother"
    smoother.write_outputs(
        baseline_result,
        baseline_config,
        baseline_dir,
        position_path=position_path,
        device_path=device_path,
        skip_epochs=1,
        leap_seconds=18,
    )
    candidate_results: dict[str, smoother.SmoothingResult] = {}
    candidate_dirs: dict[str, Path] = {"existing-smoother": baseline_dir}
    for candidate in CANDIDATES:
        config = smoother.SmootherConfig(
            BASELINE_CONFIG["process_noise"],
            BASELINE_CONFIG["measurement_floor_m"],
            BASELINE_CONFIG["outlier_gate_sigma"],
            BASELINE_CONFIG["segment_gap_s"],
            int(candidate["max_consecutive_rejects"]),
            float(candidate["max_prediction_duration_s"]),
        )
        result = smoother.smooth_positions(positions, epochs, config)
        candidate_id = str(candidate["id"])
        candidate_results[candidate_id] = result
        candidate_dir = route_output / candidate_id
        candidate_dirs[candidate_id] = candidate_dir
        smoother.write_outputs(
            result,
            config,
            candidate_dir,
            position_path=position_path,
            device_path=device_path,
            skip_epochs=1,
            leap_seconds=18,
        )
    return {
        "dataset_id": dataset_id,
        "position": position_path,
        "device": device_path,
        "truth": truth_path,
        "positions": positions,
        "epochs": epochs,
        "baseline": baseline_result,
        "candidates": candidate_results,
        "output_dirs": candidate_dirs,
        "input_hashes": {
            "position": _sha256(position_path),
            "device": _sha256(device_path),
        },
        "stages": stages or [],
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


def _candidate_report(
    train_scores: dict[str, dict[str, dict[str, Any]]],
    validation_scores: dict[str, dict[str, Any]],
    validation_raw: dict[str, dict[str, Any]],
    validation_existing: dict[str, dict[str, Any]],
    main_scores: dict[str, dict[str, Any]],
    main_existing: dict[str, dict[str, Any]],
    main_byte_identity: dict[str, bool],
    main_reacquisitions: dict[str, int],
) -> tuple[dict[str, Any], str | None]:
    existing_train = previous._aggregate(
        [train_scores["existing-smoother"][dataset_id] for dataset_id in TRAIN_IDS]
    )
    values: dict[str, Any] = {}
    for candidate in CANDIDATES:
        candidate_id = str(candidate["id"])
        train_aggregate = previous._aggregate(
            [train_scores[candidate_id][dataset_id] for dataset_id in TRAIN_IDS]
        )
        eligible, failures = previous._aggregate_non_regression(
            train_aggregate, existing_train
        )
        values[candidate_id] = {
            "candidate": candidate,
            "train_aggregate": train_aggregate,
            "train_non_regression_vs_existing": eligible,
            "train_non_regression_failures": failures,
            "new_validation": validation_scores[candidate_id][NEW_VALIDATION_ID],
            "development_main": main_scores[candidate_id][MAIN_ID],
            "main_pos_byte_identical": main_byte_identity[candidate_id],
            "main_reacquisition_count": main_reacquisitions[candidate_id],
        }
    eligible_values = [value for value in values.values() if value["train_non_regression_vs_existing"]]
    selected = min(eligible_values, key=previous._candidate_rank) if eligible_values else None
    selected_id = str(selected["candidate"]["id"]) if selected else None
    validation_gate = None
    main_gate = None
    if selected_id is not None:
        validation_gate = previous._compare_metrics(
            validation_scores[selected_id][NEW_VALIDATION_ID],
            {
                "raw_spp_hatch": validation_raw[NEW_VALIDATION_ID],
                "existing_smoother": validation_existing[NEW_VALIDATION_ID],
            },
        )
        validation_gate["strict_h_p95_improvement_vs_existing"] = (
            previous._metric(
                validation_scores[selected_id][NEW_VALIDATION_ID],
                ("horizontal_wgs84_m", "p95_m"),
            )
            < previous._metric(
                validation_existing[NEW_VALIDATION_ID],
                ("horizontal_wgs84_m", "p95_m"),
            )
            - 1e-12
        )
        main_gate = previous._compare_metrics(
            main_scores[selected_id][MAIN_ID],
            {"existing_smoother": main_existing[MAIN_ID]},
        )
        main_gate["pos_byte_identical"] = main_byte_identity[selected_id]
        main_gate["reacquisition_count_zero"] = main_reacquisitions[selected_id] == 0
        main_gate["failures_by_reference"].setdefault("main_contract", [])
        if not main_gate["pos_byte_identical"]:
            main_gate["failures_by_reference"]["main_contract"].append("pos_not_byte_identical")
        if not main_gate["reacquisition_count_zero"]:
            main_gate["failures_by_reference"]["main_contract"].append("reacquisition_fired")
        main_gate["non_regression_passed"] = bool(
            main_gate["non_regression_passed"]
            and main_gate["pos_byte_identical"]
            and main_gate["reacquisition_count_zero"]
        )
    promotion = "no-go-no-eligible-train-candidate"
    if selected_id is not None:
        if not validation_gate["non_regression_passed"]:
            promotion = "no-go-new-validation-regression"
        elif not validation_gate["strict_h_p95_improvement_vs_existing"]:
            promotion = "no-go-new-validation-no-p95-improvement"
        elif not main_gate["non_regression_passed"]:
            promotion = "no-go-development-main-contract-or-regression"
        else:
            promotion = "promote-development-only"
    return (
        {
            "existing_train_aggregate": existing_train,
            "candidates": values,
            "selected_candidate_id": selected_id,
            "new_validation_gate": validation_gate,
            "development_main_gate": main_gate,
            "promotion_decision": promotion,
            "existing_smoother_retained": promotion != "promote-development-only",
        },
        selected_id,
    )


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-reacquisition-conservative-eval")
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
            raise ConservativeEvaluationError("max-epochs must be -1 or a positive integer")
        selection_record = _load_selection_record(args.selection_record)
        profile = previous._load_profile(args.profile)
        inventory = generalization.inventory_archive(args.archive)
        candidate, signal_screen = _select_new_validation(
            args.archive,
            inventory,
            profile,
            selection_record,
            args.generalization_root,
        )
        archive_expected = str(dict(profile["archive"]).get("sha256", ""))
        if not archive_expected:
            raise ConservativeEvaluationError("profile archive SHA-256 is empty")
        archive_sha = _sha256(args.archive)
        if archive_sha != archive_expected:
            raise ConservativeEvaluationError("GSDC archive hash does not match frozen profile")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        inventory["selection"] = {
            "fixed_candidate_train_ids": list(TRAIN_IDS),
            "fixed_new_validation_id": NEW_VALIDATION_ID,
            "development_main_id": MAIN_ID,
            "previous_validation_not_reused": list(previous.VALIDATION_IDS),
            "development_excluded": str(dict(profile["datasets"]["development"])["id"]),
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
        candidate_profile, materialized, profile_path = _materialize_new_validation(
            args.archive,
            profile,
            candidate,
            archive_sha,
            args.output_dir,
        )
        generated = _run_adapter_and_spp(
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
        route_results[NEW_VALIDATION_ID] = _filter_route(
            NEW_VALIDATION_ID,
            Path(generated["position"]),
            Path(generated["device"]),
            Path(generated["truth"]),
            args.output_dir,
            stages=generated["stages"],
            extra={
                "truth_free_pipeline": True,
                "materialization": materialized,
                "profile": profile_path,
                "adapter_summary": generated["adapter_summary"],
                "spp_summary": generated["spp_summary"],
            },
        )
        route_results[MAIN_ID] = _filter_route(
            MAIN_ID,
            args.main_position,
            args.main_device_gnss,
            args.main_ground_truth,
            args.output_dir,
        )

        # Ground truth is parsed only after every route has published its
        # truth-free adapter/SPP/smoother artifacts.
        train_scores: dict[str, dict[str, dict[str, Any]]] = {
            "raw_spp_hatch": _score_group(route_results, TRAIN_IDS),
            "existing-smoother": _score_group(
                route_results, TRAIN_IDS, candidate_id="existing-smoother"
            ),
        }
        for candidate_spec in CANDIDATES:
            candidate_id = str(candidate_spec["id"])
            train_scores[candidate_id] = _score_group(
                route_results, TRAIN_IDS, candidate_id=candidate_id
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
        baseline_pos = route_results[MAIN_ID]["output_dirs"]["existing-smoother"] / "smoothed.pos"
        main_byte_identity = {
            str(candidate_spec["id"]): (
                (route_results[MAIN_ID]["output_dirs"][str(candidate_spec["id"])] / "smoothed.pos").read_bytes()
                == baseline_pos.read_bytes()
            )
            for candidate_spec in CANDIDATES
        }
        main_reacquisitions = {
            str(candidate_spec["id"]): len(
                route_results[MAIN_ID]["candidates"][str(candidate_spec["id"])].reacquisition_indices
            )
            for candidate_spec in CANDIDATES
        }
        selection, selected_id = _candidate_report(
            train_scores,
            validation_scores,
            validation_raw,
            validation_existing,
            main_scores,
            main_existing,
            main_byte_identity,
            main_reacquisitions,
        )

        report_path = args.output_dir / "reacquisition_conservative_report.json"
        manifest_path = args.output_dir / "reacquisition_conservative_manifest.json"
        route_report: dict[str, Any] = {}
        for dataset_id, route in route_results.items():
            route_report[dataset_id] = {
                "inputs": {
                    "position": str(route["position"]),
                    "device_gnss": str(route["device"]),
                    "ground_truth": str(route["truth"]),
                    "sha256": route["input_hashes"],
                },
                "artifacts": {
                    name: _artifact(path / "smoother_manifest.json")
                    for name, path in route["output_dirs"].items()
                },
                "reacquisition": {
                    "existing_smoother": {
                        "maximum_observed_reject_run": route["baseline"].max_consecutive_rejects,
                        "maximum_observed_prediction_duration_s": route["baseline"].max_prediction_duration_s,
                        "reacquisition_count": 0,
                    },
                    **{
                        candidate_id: {
                            "count": len(result.reacquisition_indices),
                            "maximum_observed_reject_run": result.max_consecutive_rejects,
                            "maximum_observed_prediction_duration_s": result.max_prediction_duration_s,
                        }
                        for candidate_id, result in route["candidates"].items()
                    }
                },
                "truth_free_pipeline": route["extra"].get("truth_free_pipeline", False),
                "stages": route["stages"],
            }
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-conservative-reacquisition-evaluation",
            "selection_record": str(args.selection_record),
            "archive": {
                "path": str(args.archive),
                "sha256": archive_sha,
                "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path)},
            },
            "roles": {
                "candidate_train": list(TRAIN_IDS),
                "new_validation": [NEW_VALIDATION_ID],
                "development_main_regression": [MAIN_ID],
                "previous_validation_not_reused": list(previous.VALIDATION_IDS),
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
                "new_validation_signal_only_screen": signal_screen,
                "main_pos_byte_identity_required": True,
                "imu_adaptive_status": "No-Go retained; not evaluated or changed",
            },
            "materialization": {
                "manifest": str(materialized["manifest"]),
                "manifest_sha256": materialized["manifest_sha256"],
                "candidate_profile": {"path": str(profile_path), "sha256": _sha256(profile_path)},
                "truth_used_for_materialization": False,
            },
            "routes": route_report,
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
            "schema_version": "smartphone-r5-reacquisition-conservative-manifest.v1",
            "report": _artifact(report_path),
            "selection_record": {"path": str(args.selection_record), "sha256": _sha256(args.selection_record)},
            "inventory": _artifact(inventory_path),
            "new_validation_materialization": {
                "path": str(materialized["manifest"]),
                "sha256": _sha256(materialized["manifest"]),
            },
            "route_artifacts": {
                dataset_id: {
                    name: _artifact(path / "smoother_manifest.json")
                    for name, path in route["output_dirs"].items()
                }
                for dataset_id, route in route_results.items()
            },
            "truth_free_pipeline": {
                "new_validation_adapter_summary": _artifact(generated["adapter_summary"]),
                "new_validation_spp_summary": _artifact(generated["spp_summary"]),
            },
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "holdout_materialized": False,
        }
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone conservative reacquisition evaluation complete: {report_path}")
        return 0
    except (
        OSError,
        ValueError,
        KeyError,
        TypeError,
        json.JSONDecodeError,
        zipfile.BadZipFile,
        ConservativeEvaluationError,
        generalization.GeneralizationError,
        previous.ReacquisitionError,
        smoother.SmootherError,
    ) as exc:
        print(f"Smartphone conservative reacquisition evaluation failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(run())
