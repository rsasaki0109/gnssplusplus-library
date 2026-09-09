#!/usr/bin/env python3
"""Phase183 one-shot H accuracy evaluator for the sealed Phase182 output.

Verification mode reads only source and sealed metadata.  ``--evaluate`` is
the single truth-authorized mode: it reads the one immutable Phase182 H CSV
and the one pinned H development truth CSV, delegates parsing/join/scoring to
the pinned Phase142/Phase82/Phase76 implementation, and writes aggregate
metrics without copying coordinate rows into the result.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
TESTS = ROOT / "tests/test_smartphone_phase183_phase182_accuracy.py"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase183_phase182_accuracy_manifest_v1.json"
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase183_phase182_accuracy_result_v1.json"
PHASE182_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase182_h_native_raw_p_no_doppler_imu_main_result_v1.json"
PHASE182_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase182_h_native_raw_p_no_doppler_imu_main_manifest_v1.json"
PHASE37_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase37_pixel5_repeatability_freeze_v1.json"
PHASE44_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_freeze_v1.json"
PHASE44_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase44_pixel5_development_accuracy_result_v1.json"
ARCHIVE_INVENTORY = ROOT / "output/smartphone-r5/generalization-v6/archive_inventory.json"
PHASE142 = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase142_phase141_accuracy.py"
PHASE82 = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE76 = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
PHASE137 = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase137_phase136_accuracy.py"
PHASE134 = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_accuracy.py"

ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"
LABEL = "MTV-H"
CANDIDATE_ROWS = 3139
MODELED_EPOCHS = 3140
TRUTH_ROWS = 3139
THRESHOLD = 0.782

PHASE182_RESULT_SHA = "1ea944668d9af9cfbcc2fdb2539172a21a4abd15f6674adfdd631133de104843"
PHASE182_RESULT_COMMIT = "e9becbb0b0387551d6024677c8fb01b240dd9140"
PHASE182_MANIFEST_SHA = "cc873daa97bfcbaa32bd1439a49bcc47a010bb68b54cc1846e62f142eabc1756"
PHASE182_MANIFEST_COMMIT = "52ec0e43f1d5fc9396ed9fe9546240446fe1db1b"
PHASE37_SHA = "629689a206e59655c7cafbc59b6011553fa1953381df74d644bfd567708be8fd"
PHASE37_COMMIT = "fc47084edde5217bfb933729a58344408f994081"
PHASE44_SHA = "95c0990af0015b7cb5fcf736aefbcff6fc97356093edcf03094b31b4083b28bc"
PHASE44_COMMIT = "e33c198745b0fc94ab425b9e57bf9663c67c60f8"
PHASE44_RESULT_SHA = "9e441c78f7c2bf8b3cf2cf9a7c9fe7e447fb2ff8eb6324fa5ef138c3b419e48a"
PHASE44_RESULT_COMMIT = "a323eb621d2cd3d13b2d298d6c5a57dd3c681638"
ARCHIVE_INVENTORY_SHA = "f4e68109885eecfc14b2bd5e8fab87e18d73473c04bb7f53da31b7040a8e90a7"
PHASE142_SHA = "03253d1f7c70dc761699ce19d03123457e386b6ab9cd0facb395c942f8450fb6"
PHASE82_SHA = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE76_SHA = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
PHASE137_SHA = "f294aecadca1e427673ada124f24f91bb5cb2b880156d8337fde8f7afc32c200"
PHASE134_SHA = "b554e2c07bedffcb53751e4e6351bcf5006fe99370dee34a16cc4f3eb81b145a"
PHASE142_COMMIT = "adbce9bd266922bb584ffb40db7134790c71be07"
PHASE82_COMMIT = "be45879297a2a00a8574c77b71be0a5b721ff8c9"
PHASE76_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
PHASE137_COMMIT = "97bc75d4e94b339507864a7b7c9f188ae33915fd"
PHASE134_COMMIT = "b59ca771dabcc1185910d3d95259572f76b16869"
H_TRUTH_SHA = "a55f452e611426693677fbeacde227fc80e5f03f6040d04aef9d6a2baf08d249"
H_TRUTH_BYTES = 305432
H_TRUTH_PATH = "output/smartphone-r5/phase44-pixel5-development-accuracy-v1/truth/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv"
H_TRUTH_ARCHIVE_MEMBER = "dataset_2023/train/2021-08-24-20-32-us-ca-mtv-h/pixel5/ground_truth.csv"
H_TRUTH_ARCHIVE_CRC32 = "fe212a4d"
H_CANDIDATE_SHA = "f40d93300075d99844fc3d74b66bd3558376ac6f247fc22731d917467ecf5873"
H_CANDIDATE_BYTES = 251174
H_CANDIDATE_PATH = "output/smartphone-r5/phase182-h-native-raw-p-no-doppler-imu-main-v1/mtv-h/opaque_solution_output.csv"

SCHEMA = "smartphone-r5-phase183-phase182-accuracy-manifest.v1"
RESULT_SCHEMA = "smartphone-r5-phase183-phase182-accuracy-result.v1"


class Phase183Error(ValueError):
    """Fail-closed Phase183 contract violation."""


def fail(message: str) -> Phase183Error:
    return Phase183Error(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=_no_duplicates)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase183Error) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def static_hash(path: Path, label: str) -> str:
    lowered = str(path).lower()
    if path.name.lower() in {"opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs"} or "/truth/" in lowered or ".mat" in lowered or ".pdc" in lowered or "kaggle" in lowered:
        raise fail(f"payload hash forbidden before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def deferred(value: Any, basename: str, fragment: str, label: str) -> str:
    if not isinstance(value, str) or not value or Path(value).is_absolute() or ".." in Path(value).parts or Path(value).name != basename or fragment not in value:
        raise fail(f"unsafe deferred {label} path: {value!r}")
    return value


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load pinned source: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def metric_reference() -> tuple[Any, Any]:
    equal(static_hash(PHASE142, "Phase142 evaluator"), PHASE142_SHA, "Phase142 hash")
    equal(static_hash(PHASE82, "Phase82 scorer"), PHASE82_SHA, "Phase82 hash")
    equal(static_hash(PHASE76, "Phase76 parser"), PHASE76_SHA, "Phase76 hash")
    equal(static_hash(PHASE137, "Phase137 metric source"), PHASE137_SHA, "Phase137 hash")
    equal(static_hash(PHASE134, "Phase134 metric source"), PHASE134_SHA, "Phase134 hash")
    reference = load_module(PHASE142, "phase142_reference_for_phase183")
    return reference, reference.load_phase82()


def metric_contract() -> dict[str, Any]:
    reference, _ = metric_reference()
    return reference.metric_contract()


def verify_candidate_metadata() -> dict[str, Any]:
    result = read_json(PHASE182_RESULT, "Phase182 result metadata")
    equal(static_hash(PHASE182_RESULT, "Phase182 result"), PHASE182_RESULT_SHA, "Phase182 result hash")
    equal(result.get("schema_version"), "smartphone-r5-phase182-h-native-raw-p-no-doppler-imu-main-result.v1", "Phase182/result schema")
    equal(result.get("status"), "completed-gnss-first-and-imu-main-native-output", "Phase182/status")
    execution = result.get("execution")
    if not isinstance(execution, Mapping):
        raise fail("Phase182 execution metadata missing")
    equal(execution.get("return_code"), 0, "Phase182/return code")
    equal(execution.get("opaque_solution_output_present"), True, "Phase182/candidate present")
    equal(execution.get("opaque_solution_output_sha256"), H_CANDIDATE_SHA, "Phase182/candidate hash")
    equal(execution.get("opaque_solution_output_bytes"), H_CANDIDATE_BYTES, "Phase182/candidate bytes")
    coverage = result.get("coverage")
    if not isinstance(coverage, Mapping):
        raise fail("Phase182 coverage metadata missing")
    equal(coverage.get("modeled_epochs"), MODELED_EPOCHS, "Phase182/modeled epochs")
    equal(coverage.get("published_solution_rows"), CANDIDATE_ROWS, "Phase182/published rows")
    equal(coverage.get("warmup_epoch_excluded"), True, "Phase182/warmup exclusion")
    equal(coverage.get("interpolated_epochs"), 0, "Phase182/interpolation")
    equal(coverage.get("edge_hold_epochs"), 0, "Phase182/edge hold")
    equal(coverage.get("unresolved_epochs"), 0, "Phase182/unresolved")
    recipe = result.get("recipe")
    if not isinstance(recipe, Mapping):
        raise fail("Phase182 recipe metadata missing")
    equal(recipe.get("phase94_selector"), False, "Phase182/Phase94 selector")
    equal(recipe.get("pixel5_offset_reapplication", False), False, "Phase182/Pixel5 offset")
    manifest = read_json(PHASE182_MANIFEST, "Phase182 manifest metadata")
    equal(static_hash(PHASE182_MANIFEST, "Phase182 manifest"), PHASE182_MANIFEST_SHA, "Phase182 manifest hash")
    equal(manifest.get("phase"), 182, "Phase182 manifest phase")
    equal(manifest.get("selector_contract", {}).get("pixel5_offset_reapplication"), False, "Phase182 manifest offset")
    return {
        "path": H_CANDIDATE_PATH,
        "sha256": H_CANDIDATE_SHA,
        "bytes": H_CANDIDATE_BYTES,
        "rows": CANDIDATE_ROWS,
        "modeled_epochs": MODELED_EPOCHS,
        "warmup_epoch_excluded": True,
        "pixel5_offset_reapplication": 0,
    }


def verify_truth_provenance() -> dict[str, Any]:
    phase37 = read_json(PHASE37_FREEZE, "Phase37 truth provenance")
    equal(static_hash(PHASE37_FREEZE, "Phase37 freeze"), PHASE37_SHA, "Phase37 freeze hash")
    equal(phase37.get("schema_version"), "smartphone-r5-phase37-pixel5-repeatability-freeze.v1", "Phase37/schema")
    member = phase37.get("selected_archive_members", {}).get(ROUTE, {}).get("ground_truth")
    if not isinstance(member, Mapping):
        raise fail("Phase37 H archive truth member missing")
    equal(member.get("name"), H_TRUTH_ARCHIVE_MEMBER, "Phase37/H archive member")
    equal(member.get("file_size"), H_TRUTH_BYTES, "Phase37/H archive bytes")
    equal(member.get("crc32_hex"), H_TRUTH_ARCHIVE_CRC32, "Phase37/H archive CRC")
    phase44 = read_json(PHASE44_FREEZE, "Phase44 truth provenance")
    equal(static_hash(PHASE44_FREEZE, "Phase44 freeze"), PHASE44_SHA, "Phase44 freeze hash")
    equal(phase44.get("schema_version"), "smartphone-r5-phase44-pixel5-development-accuracy-freeze.v1", "Phase44/schema")
    equal(phase44.get("status"), "frozen-before-truth-materialization-and-read", "Phase44/status")
    member44 = phase44.get("archive", {}).get("added_ground_truth_members", {}).get(ROUTE)
    equal(member44, dict(member), "Phase44/archive member")
    cohort = phase44.get("cohort")
    if not isinstance(cohort, Mapping):
        raise fail("Phase44 cohort missing")
    if ROUTE not in cohort.get("route_order", []) or ROUTE not in cohort.get("added_identities", []):
        raise fail("Phase44 H route role missing")
    equal(cohort.get("fresh_validation", {}).get("opened"), False, "Phase44/fresh validation")
    equal(cohort.get("future_holdout", {}).get("opened"), False, "Phase44/future holdout")
    inventory = read_json(ARCHIVE_INVENTORY, "archive inventory metadata")
    equal(static_hash(ARCHIVE_INVENTORY, "archive inventory"), ARCHIVE_INVENTORY_SHA, "archive inventory hash")
    inventory_member: Mapping[str, Any] | None = None
    def find_inventory(value: Any) -> None:
        nonlocal inventory_member
        if inventory_member is not None:
            return
        if isinstance(value, Mapping):
            if value.get("dataset_id") == ROUTE:
                files = value.get("ground_truth.csv")
                if not isinstance(files, Mapping):
                    files = value.get("central_directory_files", {}).get("ground_truth.csv") if isinstance(value.get("central_directory_files"), Mapping) else None
                candidate = files
                if isinstance(candidate, Mapping):
                    inventory_member = candidate
            for child in value.values():
                find_inventory(child)
        elif isinstance(value, list):
            for child in value:
                find_inventory(child)
    find_inventory(inventory)
    if inventory_member is None:
        raise fail("archive inventory H truth member missing")
    equal(inventory_member.get("name"), H_TRUTH_ARCHIVE_MEMBER, "archive inventory/H member")
    equal(inventory_member.get("file_size"), H_TRUTH_BYTES, "archive inventory/H bytes")
    equal(inventory_member.get("crc32_hex"), H_TRUTH_ARCHIVE_CRC32, "archive inventory/H CRC")
    phase44_result = read_json(PHASE44_RESULT, "Phase44 result provenance")
    equal(static_hash(PHASE44_RESULT, "Phase44 result"), PHASE44_RESULT_SHA, "Phase44 result hash")
    result_truth = phase44_result.get("routes", {}).get(ROUTE, {}).get("truth")
    if not isinstance(result_truth, Mapping):
        raise fail("Phase44 H truth result metadata missing")
    equal(result_truth.get("member"), H_TRUTH_ARCHIVE_MEMBER, "Phase44 result/H member")
    equal(result_truth.get("path"), H_TRUTH_PATH, "Phase44 result/H path")
    equal(result_truth.get("sha256"), H_TRUTH_SHA, "Phase44 result/H hash")
    equal(result_truth.get("bytes"), H_TRUTH_BYTES, "Phase44 result/H bytes")
    equal(result_truth.get("rows"), TRUTH_ROWS, "Phase44 result/H rows")
    return {
        "path": H_TRUTH_PATH,
        "sha256": H_TRUTH_SHA,
        "bytes": H_TRUTH_BYTES,
        "rows": TRUTH_ROWS,
        "expected_missing_truth_key": None,
        "role": "development/train added Pixel5 route; not heldout or leaderboard proof",
        "archive_member": H_TRUTH_ARCHIVE_MEMBER,
        "archive_crc32": H_TRUTH_ARCHIVE_CRC32,
        "prior_phase44_materialization": True,
    }


def verify_manifest() -> dict[str, Any]:
    manifest = read_json(MANIFEST, "Phase183 manifest")
    for key, expected in (("schema_version", SCHEMA), ("phase", 183), ("execution_label", "Luna Max"), ("status", "sealed-before-phase183-truth-pass"), ("route_order", [ROUTE])):
        equal(manifest.get(key), expected, f"manifest/{key}")
    equal(manifest.get("metric_contract"), metric_contract(), "manifest/metric contract")
    candidate = verify_candidate_metadata()
    equal(manifest.get("candidate_reference"), {**candidate, "source": "Phase182 opaque H output", "opaque_only": True, "route": ROUTE}, "manifest/candidate reference")
    truth = verify_truth_provenance()
    equal(manifest.get("truth_reference"), {**truth, "source": "Phase44 frozen development truth; Phase37 official archive provenance"}, "manifest/truth reference")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest/authority missing")
    pins = {
        "phase182_result": (PHASE182_RESULT, PHASE182_RESULT_SHA, PHASE182_RESULT_COMMIT),
        "phase182_manifest": (PHASE182_MANIFEST, PHASE182_MANIFEST_SHA, PHASE182_MANIFEST_COMMIT),
        "phase37_freeze": (PHASE37_FREEZE, PHASE37_SHA, PHASE37_COMMIT),
        "phase44_freeze": (PHASE44_FREEZE, PHASE44_SHA, PHASE44_COMMIT),
        "phase44_result": (PHASE44_RESULT, PHASE44_RESULT_SHA, PHASE44_RESULT_COMMIT),
        "archive_inventory": (ARCHIVE_INVENTORY, ARCHIVE_INVENTORY_SHA, None),
        "phase142_evaluator": (PHASE142, PHASE142_SHA, PHASE142_COMMIT),
        "phase82_scorer": (PHASE82, PHASE82_SHA, PHASE82_COMMIT),
        "phase76_parser": (PHASE76, PHASE76_SHA, PHASE76_COMMIT),
        "phase137_metric_source": (PHASE137, PHASE137_SHA, PHASE137_COMMIT),
        "phase134_metric_source": (PHASE134, PHASE134_SHA, PHASE134_COMMIT),
        "evaluator": (EVALUATOR, manifest["authority"]["evaluator"]["sha256"], None),
        "focused_tests": (TESTS, manifest["authority"]["focused_tests"]["sha256"], None),
    }
    for key, (path, digest, commit) in pins.items():
        item = authority.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest authority/{key} missing")
        equal(item.get("path"), relative(path), f"authority/{key}/path")
        equal(item.get("sha256"), digest, f"authority/{key}/sha256")
        equal(static_hash(path, f"authority/{key}"), digest, f"authority/{key}/current hash")
        if commit is not None:
            equal(item.get("commit"), commit, f"authority/{key}/commit")
    equal(manifest.get("comparison_baselines"), {"phase118_champion_macro_m": 0.8141981503, "phase142_macro_m": 0.8142362396358963, "informational_only": True, "not_comparable_to_single_route_h": True}, "manifest/baselines")
    equal(manifest.get("read_accounting_before_truth"), {"candidate_payload_reads": 0, "truth_payload_reads": 0, "candidate_coordinate_interpretations": 0, "accuracy_calculations": 0, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0}, "manifest/pretruth accounting")
    policy = manifest.get("execution_policy")
    if not isinstance(policy, Mapping):
        raise fail("manifest/execution policy missing")
    for key, expected in (("candidate_reads", 1), ("truth_reads", 1), ("accuracy_calculations", 1), ("native_solver_invocations", 0), ("raw_reads", 0), ("mat_pdc_precomputed_coordinate_reads", 0), ("kaggle_or_token_access", 0), ("reruns", 0), ("fallbacks", 0), ("repairs", 0), ("solution_publication", False)):
        equal(policy.get(key), expected, f"manifest/policy/{key}")
    return manifest


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < THRESHOLD


def route_gates(ordered_count: int, score: Mapping[str, Any]) -> dict[str, bool]:
    return {
        "candidate_schema_and_exact_alignment": ordered_count == CANDIDATE_ROWS,
        "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
        "truth_domain_coverage_exact": score.get("truth_row_coverage") == 1.0 and score.get("matched_rows") == TRUTH_ROWS,
        "candidate_all_finite_and_earth_valid": score.get("finite") is True,
        "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
        "candidate_route_score_finite": finite(score.get("score_m")),
    }


def read_candidate_once(path: Path, pin: Mapping[str, Any], p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail("candidate seal mismatch")
    ordered, mapping = p82.P76.P74._parse_submission(payload, ROUTE)
    if len(ordered) != CANDIDATE_ROWS or len(mapping) != len(ordered):
        raise fail("candidate exact row alignment failed")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail("candidate finite/Earth-valid preflight failed")
    return ordered, mapping, {"rows": len(ordered), "bytes": len(payload), "sha256": digest, "read_count": 1, "coordinate_rows_omitted": True}


def read_truth_once(path: Path, pin: Mapping[str, Any], p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail("truth seal mismatch")
    truth = p82.P76._parse_truth_dictreader(payload, ROUTE)
    if len(truth) != TRUTH_ROWS:
        raise fail("truth exact row count failed")
    return truth, {"rows": len(truth), "bytes": len(payload), "sha256": digest, "read_count": 1, "coordinate_rows_omitted": True}


def evaluate(result_path: Path = RESULT) -> dict[str, Any]:
    if result_path.exists():
        raise fail(f"refusing to overwrite existing result: {result_path}")
    manifest = verify_manifest()
    reference, p82 = metric_reference()
    candidate_pin = manifest["candidate_reference"]
    truth_pin = manifest["truth_reference"]
    candidate_path = ROOT / deferred(candidate_pin["path"], "opaque_solution_output.csv", "/phase182-h-native-raw-p-no-doppler-imu-main-v1/", "candidate")
    ordered, candidate, candidate_meta = read_candidate_once(candidate_path, candidate_pin, p82)
    truth_path = ROOT / deferred(truth_pin["path"], "ground_truth.csv", "/phase44-pixel5-development-accuracy-v1/truth/", "truth")
    truth, truth_meta = read_truth_once(truth_path, truth_pin, p82)
    score = p82.P76._score_prediction(candidate, truth, truth_pin.get("expected_missing_truth_key"), ROUTE, ordered)
    checks = route_gates(len(ordered), score)
    macro = score["score_m"]
    gates = {
        "exact_one_route_one_evaluation": all((candidate_meta["read_count"], truth_meta["read_count"], 1) == (1, 1, 1) for _ in [0]),
        **checks,
        "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(macro),
        "pixel5_offset_already_exactly_once_no_reapplication": candidate_pin["pixel5_offset_reapplication"] == 0,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 183,
        "execution_label": "Luna Max",
        "status": "go-phase183-h-accuracy" if not failed else "no-go-phase183-h-accuracy",
        "decision": "H development/train accuracy diagnostic only; not heldout/leaderboard proof and no submission authorized",
        "candidate_source": "Phase182 opaque H output; evaluator input only",
        "route": LABEL,
        "dataset_id": ROUTE,
        "candidate": {"sha256": candidate_meta["sha256"], "bytes": candidate_meta["bytes"], "rows": candidate_meta["rows"], "read_count": 1},
        "truth": {"sha256": truth_meta["sha256"], "bytes": truth_meta["bytes"], "rows": truth_meta["rows"], "read_count": 1, "role": truth_pin["role"]},
        "metrics": {"matched_rows": score["matched_rows"], "prediction_rows": score["prediction_rows"], "truth_rows": score["truth_rows"], "missing_prediction_rows": score["prediction_rows"] - score["matched_rows"], "missing_truth_rows": score["truth_rows"] - score["matched_rows"], "prediction_domain_coverage": score["prediction_domain_coverage"], "truth_domain_coverage": score["truth_row_coverage"], "p50_m": score["p50_m"], "p95_m": score["p95_m"], "route_score_m": score["score_m"], "over_70_mps_count": score["over_70_mps_count"], "finite": score["finite"], "interpolated_rows": 0, "edge_hold_rows": 0, "extrapolated_rows": 0},
        "coverage_policy": {"modeled_epochs": MODELED_EPOCHS, "published_prediction_rows": CANDIDATE_ROWS, "warmup_epoch_excluded": True, "truth_rows": TRUTH_ROWS, "missing_truth_rows": score["truth_rows"] - score["matched_rows"], "interpolated_rows": 0, "edge_hold_rows": 0, "extrapolated_rows": 0},
        "aggregate": {"candidate_route_score_m": macro, "route_order": [ROUTE], "macro_weighting": "single-route scalar; no cross-route macro claim", "historical_phase118_m": 0.8141981503, "historical_phase142_m": 0.8142362396358963, "historical_baselines_comparable": False},
        "metric_contract": manifest["metric_contract"],
        "gates": gates,
        "failed_gates": failed,
        "strict_0_782_gate": {"candidate_macro_score_m": macro, "comparator": "candidate_macro_score_m < 0.782", "threshold_m": THRESHOLD, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]},
        "read_accounting": {"candidate_reads": 1, "truth_reads": 1, "accuracy_calculations": 1, "candidate_coordinate_interpretations": 1, "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0},
        "solution_rows_in_result": False,
        "release_or_submission_authorized": False,
    }
    descriptor, temporary = tempfile.mkstemp(prefix=f".{result_path.name}.", suffix=".tmp", dir=result_path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(result, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, result_path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise
    return result


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT)
    args = parser.parse_args(argv)
    if int(args.verify_manifest) + int(args.evaluate) != 1:
        parser.error("choose exactly one mode")
    try:
        if args.verify_manifest:
            verify_manifest()
            return 0
        result = evaluate(args.result_json)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 1
    except Exception as exc:
        print(f"phase183 fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
