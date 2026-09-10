#!/usr/bin/env python3
"""One-shot v2 audit of the fixed WLS residual median5/zero-shift lane.

This command deliberately does not search candidates.  ``median5_shift_0``
and the v2 public-metric promotion gate are frozen in a metadata-only record
before the fresh validation payload is opened.  The candidate is generated
after the native Galileo-E1/Hatch30 lane and the v3 stability selector, and
the selected validation truth is materialized/read exactly once afterwards.
The next holdout remains central-directory-only and is never materialized.
"""

from __future__ import annotations

import argparse
import hashlib
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
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_generalization as generalization  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_wls_residual as residual  # noqa: E402
import gnss_smartphone_wls_residual_eval as residual_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-residual-release-evaluation.v2"
MANIFEST_SCHEMA_VERSION = "smartphone-r5-wls-residual-release-manifest.v2"
SELECTION_SCHEMA_VERSION = "smartphone-r5-wls-residual-release-candidate-selection.v2"
DEFAULT_ARCHIVE = ROOT / "data" / "gsdc2023" / "cache" / "dataset_2023.zip"
DEFAULT_PROFILE = ROOT / "configs" / "benchmarks" / "smartphone_r5_gsdc2023.json"
DEFAULT_INVENTORY = (
    ROOT / "output" / "smartphone-r5" / "generalization-v6" / "archive_inventory.json"
)
DEFAULT_SELECTION_RECORD = (
    ROOT
    / "docs"
    / "use_cases"
    / "records"
    / "smartphone_r5_wls_residual_release_candidate_selection_v2.json"
)
DEFAULT_V1_REPORT = (
    ROOT / "output" / "smartphone-r5" / "wls-residual-v1" / "wls_residual_release_report.json"
)
DEFAULT_OUTPUT_DIR = ROOT / "output" / "smartphone-r5" / "wls-residual-v2"

NEW_VALIDATION_ID = "2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g"
NEXT_HOLDOUT_ID = "2023-05-25-19-10-us-ca-sjc-be2/sm-s908b"
OLD_HOLDOUT_ID = "2023-09-06-22-49-us-ca-routebb1/pixel7pro"
FIXED_CANDIDATE_ID = "median5_shift_0"
DIAGNOSTIC_KEYS = tuple(residual_eval.DIAGNOSTIC_KEYS)
EPSILON = 1.0e-12


class ResidualV2Error(ValueError):
    """Raised when the v2 fresh-validation contract is violated."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise ResidualV2Error(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    smoother._atomic_write(
        path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )


def _load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ResidualV2Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise ResidualV2Error(f"{label} must be an object: {path}")
    return payload


def _absolute(path: Path) -> Path:
    return path if path.is_absolute() else ROOT / path


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    try:
        for key in path:
            value = value[key]
        number = float(value)
    except (KeyError, TypeError, ValueError):
        return math.inf
    return number if math.isfinite(number) else math.inf


def _load_selection(path: Path) -> dict[str, Any]:
    record = _load_json(path, "v2 selection record")
    if record.get("schema_version") != SELECTION_SCHEMA_VERSION:
        raise ResidualV2Error("v2 selection schema mismatch")
    if record.get("status") != "selection-frozen-before-member-content-read":
        raise ResidualV2Error("v2 selection was not frozen before payload access")
    fixed = record.get("fixed_candidate")
    if not isinstance(fixed, dict) or fixed.get("candidate_id") != FIXED_CANDIDATE_ID:
        raise ResidualV2Error("v2 candidate is not fixed to median5 shift0")
    if fixed.get("candidate_search_after_freeze") is not False:
        raise ResidualV2Error("v2 candidate search is not sealed")
    selected = record.get("metadata_selection", {}).get("selected_validation")
    if not isinstance(selected, dict) or selected.get("dataset_id") != NEW_VALIDATION_ID:
        raise ResidualV2Error("v2 validation ID differs from selection record")
    excluded = record.get("excluded_ids", {}).get("all_previously_used_or_sealed")
    if not isinstance(excluded, list):
        raise ResidualV2Error("v2 selection lacks exclusion set")
    if any(value not in excluded for value in (OLD_HOLDOUT_ID, NEXT_HOLDOUT_ID)):
        raise ResidualV2Error("v2 selection does not exclude both holdouts")
    if record.get("sealed_data_policy", {}).get("next_holdout_materialization_forbidden") is not True:
        raise ResidualV2Error("v2 next holdout is not sealed")
    gate = record.get("v2_promotion_gate_frozen_before_truth")
    if not isinstance(gate, dict) or gate.get("selection_and_ranking_truth_free") is not True:
        raise ResidualV2Error("v2 promotion gate was not frozen truth-free")
    return record


def _verify_inventory(
    archive: Path, inventory_path: Path, record: dict[str, Any]
) -> tuple[dict[str, Any], dict[str, Any], str]:
    archive_contract = record.get("archive")
    if not isinstance(archive_contract, dict):
        raise ResidualV2Error("v2 selection lacks archive contract")
    if _sha256(inventory_path) != archive_contract.get("central_directory_inventory_sha256"):
        raise ResidualV2Error("central-directory inventory hash changed")
    inventory = _load_json(inventory_path, "central-directory inventory")
    inventory_archive = inventory.get("archive")
    if not isinstance(inventory_archive, dict) or inventory_archive.get("central_directory_only") is not True or inventory_archive.get("member_content_read") is not False:
        raise ResidualV2Error("inventory is not payload-free")
    archive_hash = _sha256(archive)
    if archive_hash != archive_contract.get("sha256"):
        raise ResidualV2Error("archive hash differs from v2 selection")
    records = {
        str(item.get("dataset_id")): item
        for item in inventory.get("train", {}).get("records", [])
        if isinstance(item, dict)
    }
    candidate = records.get(NEW_VALIDATION_ID)
    next_holdout = records.get(NEXT_HOLDOUT_ID)
    if candidate is None or next_holdout is None:
        raise ResidualV2Error("validation or next holdout is absent from inventory")
    frozen = record["metadata_selection"]["selected_validation"]
    # The shared inventory keeps the route/device members and the route-level
    # broadcast navigation member in separate fields.  The selection record
    # intentionally stores one complete required-member map, so normalize the
    # two metadata-only representations before comparing them.  This is an
    # orchestration/schema mapping only; no payload is opened here.
    candidate_members = _central_directory_members(candidate)
    if frozen.get("central_directory_members") != candidate_members:
        raise ResidualV2Error("validation central metadata changed after selection")
    for key in ("route", "phone", "calendar_year", "route_region"):
        if frozen.get(key) is not None and candidate.get(key) != frozen.get(key):
            raise ResidualV2Error(f"validation metadata changed: {key}")
    if not candidate.get("required_files_complete") or not candidate.get("broadcast_nav_present"):
        raise ResidualV2Error("validation lacks required CSV/nav members")
    if int(candidate.get("broadcast_nav_duplicate_count", 0)) != 0:
        raise ResidualV2Error("validation nav is duplicated")
    if next_holdout.get("dataset_id") != NEXT_HOLDOUT_ID:
        raise ResidualV2Error("next holdout metadata mismatch")
    return inventory, candidate, archive_hash


def _central_directory_members(candidate: dict[str, Any]) -> dict[str, Any]:
    """Return the selection-record member shape from inventory metadata only."""

    members = dict(candidate.get("central_directory_files") or {})
    broadcast_nav = candidate.get("central_directory_broadcast_nav")
    if isinstance(broadcast_nav, dict):
        members["brdc.nav"] = broadcast_nav
    return members


def _v2_gate(
    candidate: dict[str, Any],
    baseline: dict[str, Any],
    profile: dict[str, Any],
    gate_record: dict[str, Any],
) -> dict[str, Any]:
    """Apply the pre-frozen public metric gate; no candidate search occurs."""

    failures: list[str] = []
    diagnostics: dict[str, Any] = {}
    max_diag_regression = float(
        gate_record["diagnostics"].get("maximum_allowed_regression_m", 1.0e-6)
    )
    for key in DIAGNOSTIC_KEYS:
        current = _metric(candidate, ("kaggle_diagnostic_score_variants_m", key))
        reference = _metric(baseline, ("kaggle_diagnostic_score_variants_m", key))
        passed = current <= reference + max_diag_regression + EPSILON
        diagnostics[key] = {
            "candidate_m": current,
            "baseline_m": reference,
            "delta_m": current - reference,
            "passed": passed,
        }
        if not passed:
            failures.append(f"{key}_regression")
    h_p95 = _metric(candidate, ("horizontal_wgs84_m", "p95_m"))
    base_h_p95 = _metric(baseline, ("horizontal_wgs84_m", "p95_m"))
    h_p95_passed = h_p95 < base_h_p95 - EPSILON
    if not h_p95_passed:
        failures.append("h_p95_not_strictly_improved")
    h_p50 = _metric(candidate, ("horizontal_wgs84_m", "p50_m"))
    base_h_p50 = _metric(baseline, ("horizontal_wgs84_m", "p50_m"))
    h_p50_limit = float(gate_record["horizontal_p50"].get("maximum_allowed_regression_m", 0.01))
    h_p50_passed = h_p50 <= base_h_p50 + h_p50_limit + EPSILON
    if not h_p50_passed:
        failures.append("h_p50_regression_over_v2_tolerance")
    availability = _metric(candidate, ("availability_ratio",))
    base_availability = _metric(baseline, ("availability_ratio",))
    availability_passed = availability >= base_availability - EPSILON
    if not availability_passed:
        failures.append("availability_regression")
    coverage = _metric(candidate, ("truth_coverage_ratio",))
    base_coverage = _metric(baseline, ("truth_coverage_ratio",))
    coverage_passed = coverage >= base_coverage - EPSILON
    if not coverage_passed:
        failures.append("truth_coverage_regression")
    vertical = _metric(candidate, ("vertical_p95_abs_m",))
    vertical_limit = float(profile["thresholds"]["vertical_p95_max"])
    vertical_threshold_passed = vertical <= vertical_limit + EPSILON
    if not vertical_threshold_passed:
        failures.append("v_p95_profile_threshold")
    base_vertical = _metric(baseline, ("vertical_p95_abs_m",))
    vertical_major_tolerance = float(
        gate_record["vertical_p95"].get("major_regression_tolerance_m", 5.0)
    )
    vertical_major_passed = vertical <= base_vertical + vertical_major_tolerance + EPSILON
    if not vertical_major_passed:
        failures.append("v_p95_major_regression")
    return {
        "passed": not failures,
        "failures": failures,
        "diagnostics": diagnostics,
        "horizontal": {
            "h_p95_candidate_m": h_p95,
            "h_p95_baseline_m": base_h_p95,
            "h_p95_strict_improvement": h_p95_passed,
            "h_p50_candidate_m": h_p50,
            "h_p50_baseline_m": base_h_p50,
            "h_p50_allowed_regression_m": h_p50_limit,
            "h_p50_passed": h_p50_passed,
        },
        "availability": {"candidate": availability, "baseline": base_availability, "passed": availability_passed},
        "truth_coverage": {"candidate": coverage, "baseline": base_coverage, "passed": coverage_passed},
        "vertical": {
            "candidate_m": vertical,
            "baseline_m": base_vertical,
            "profile_threshold_m": vertical_limit,
            "major_regression_tolerance_m": vertical_major_tolerance,
            "within_profile_threshold": vertical_threshold_passed,
            "no_major_regression": vertical_major_passed,
        },
        "official_like_horizontal_metric": "wgs84_vincenty__linear_n_minus_1",
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-residual-v2-eval")
    )
    parser.add_argument("--archive", type=Path, default=DEFAULT_ARCHIVE)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--inventory", type=Path, default=DEFAULT_INVENTORY)
    parser.add_argument("--selection-record", type=Path, default=DEFAULT_SELECTION_RECORD)
    parser.add_argument("--v1-report", type=Path, default=DEFAULT_V1_REPORT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    return parser.parse_args(argv)


def run(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        archive = _absolute(args.archive)
        profile_path = _absolute(args.profile)
        inventory_path = _absolute(args.inventory)
        selection_path = _absolute(args.selection_record)
        v1_report_path = _absolute(args.v1_report)
        output_dir = _absolute(args.output_dir)
        if output_dir.exists() and any(output_dir.iterdir()):
            raise ResidualV2Error(f"refusing to overwrite non-empty output: {output_dir}")
        selection = _load_selection(selection_path)
        inventory, candidate, archive_hash = _verify_inventory(archive, inventory_path, selection)
        profile = residual_eval.stability_eval.previous._load_profile(profile_path)
        if profile.get("datasets", {}).get("holdout", {}).get("id") != OLD_HOLDOUT_ID:
            raise ResidualV2Error("profile holdout differs from sealed old holdout")
        v1_report = _load_json(v1_report_path, "v1 residual report")
        if v1_report.get("schema_version") != "smartphone-r5-wls-residual-release-evaluation.v1":
            raise ResidualV2Error("v1 report schema is not the audited report")
        v1_gate = selection["audit_of_v1_report"]
        output_dir.mkdir(parents=True, exist_ok=True)
        inventory_manifest = {
            "schema_version": "smartphone-r5-wls-residual-v2-central-inventory-use.v1",
            "archive": {"path": str(archive), "sha256": archive_hash, "central_directory_only": True, "member_content_read_for_next_holdout": False},
            "inventory": {"path": str(inventory_path), "sha256": _sha256(inventory_path), "record_count": len(inventory.get("train", {}).get("records", []))},
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "selected_validation": candidate,
            "next_holdout": {"dataset_id": NEXT_HOLDOUT_ID, "materialized": False, "truth_opened": False},
        }
        _atomic_json(output_dir / "archive_inventory.json", inventory_manifest)
        started = time.perf_counter()
        validation = residual_eval._validation_truth_free(
            archive,
            profile,
            candidate,
            archive_hash,
            output_dir,
            validation_id=NEW_VALIDATION_ID,
            selected_candidate_id=FIXED_CANDIDATE_ID,
            next_holdout_id=NEXT_HOLDOUT_ID,
        )
        # The v2 gate is evaluated only after truth-free generation and the
        # one permitted validation truth read.  It uses no holdout value.
        baseline = validation["scores"]["v3_selector_baseline"]
        candidate_score = validation["scores"]["residual_candidate_selector"]
        gate = _v2_gate(
            candidate_score,
            baseline,
            profile,
            selection["v2_promotion_gate_frozen_before_truth"],
        )
        selector_is_wls = validation["selector_decision"] == "wls_raw"
        if not selector_is_wls:
            gate["failures"].append("selector_did_not_choose_wls_branch")
            gate["passed"] = False
        promotion = "promote-development-only-wls-branch-postprocess" if gate["passed"] else "no-go"
        truth_free_manifest_path = output_dir / "validation" / "truth_free_manifest.json"
        truth_free_manifest = _load_json(truth_free_manifest_path, "truth-free validation manifest")
        report = {
            "schema_version": SCHEMA_VERSION,
            "decision": "development-only-wls-residual-v2-evaluation",
            "promotion_decision": promotion,
            "archive": {"path": str(archive), "sha256": archive_hash},
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "v1_audit": {
                "report": _artifact(v1_report_path),
                "known_gate_failure_cause": v1_gate["known_gate_failure_cause"],
                "comparator_corrected_before_v2": True,
                "algorithm_parameter_changed": False,
            },
            "roles": {
                "fresh_validation": NEW_VALIDATION_ID,
                "next_holdout": NEXT_HOLDOUT_ID,
                "old_holdout_excluded": OLD_HOLDOUT_ID,
            },
            "fixed_candidate": {
                "candidate_id": FIXED_CANDIDATE_ID,
                "robust_estimator": "component-wise centered median",
                "robust_window_epochs": 5,
                "along_track_shift_m": 0.0,
                "search_performed": False,
                "truth_free": True,
            },
            "truth_free_contract": {
                "next_holdout_materialized": False,
                "next_holdout_truth_opened": False,
                "validation_artifacts_sealed_before_truth": validation["truth_free_artifacts_sealed_before_truth"],
                "validation_truth_open_count": validation["truth_open_count"],
                "selector_decision": validation["selector_decision"],
                "selector_reason": validation["selector_reason"],
                "wls_branch_selected": selector_is_wls,
                "truth_free_manifest": _artifact(truth_free_manifest_path),
                "truth_free_manifest_truth_used": truth_free_manifest.get("truth_used"),
            },
            "v2_promotion_gate": {
                "gate_frozen_before_truth": selection["v2_promotion_gate_frozen_before_truth"],
                "result": gate,
            },
            "scores": validation["scores"],
            "validation": validation,
            "candidate_source_hashes": {
                "candidate_module": _sha256(ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_wls_residual.py"),
                "v2_evaluator": _sha256(Path(__file__)),
                "v1_evaluator": _sha256(ROOT / "apps" / "commands" / "benchmarks" / "gnss_smartphone_wls_residual_eval.py"),
            },
            "timing": {
                "total_wall_s": time.perf_counter() - started,
                "peak_rss_kb": validation["timing"]["peak_rss_kb"],
            },
            "promotion_scope": "development-only; production RTK/SPP default unchanged",
        }
        report_path = output_dir / "wls_residual_release_v2_report.json"
        _atomic_json(report_path, report)
        manifest = {
            "schema_version": MANIFEST_SCHEMA_VERSION,
            "truth_free_generation": True,
            "truth_free_generation_completed_before_truth": True,
            "validation_truth_open_count": validation["truth_open_count"],
            "holdout_content_opened": False,
            "holdout_truth_opened": False,
            "next_holdout_materialized": False,
            "selection_record": {"path": str(selection_path), "sha256": _sha256(selection_path)},
            "archive_inventory": _artifact(output_dir / "archive_inventory.json"),
            "truth_free_validation_manifest": _artifact(truth_free_manifest_path),
            "report": {"path": str(report_path), "sha256": _sha256(report_path)},
            "selected_lane": validation["selector_decision"],
            "selected_candidate_id": FIXED_CANDIDATE_ID,
            "promotion_decision": promotion,
        }
        manifest_path = output_dir / "wls_residual_release_v2_manifest.json"
        _atomic_json(manifest_path, manifest)
        print(f"Smartphone WLS residual v2 evaluation complete: {report_path}")
        return 0
    except (ResidualV2Error, OSError, ValueError, smoother.SmootherError) as exc:
        print(f"Smartphone WLS residual v2 evaluation failed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(run())
