#!/usr/bin/env python3
"""Launch-free contract and isolated truth-only evaluator for Phase142.

Verification modes read only tracked source and sealed JSON/Markdown metadata.
They do not materialize or probe candidate solutions, official truth, raw
inputs, native output, MAT/PDC artifacts, precomputed coordinates, or Kaggle
data.  The optional evaluation mode is gated by a separate authorization file
and is the only mode that can read one immutable candidate and one truth file
per route.  Metric/parser/join/aggregation semantics are delegated to the
pinned Phase134 implementation and checked against Phase137.
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
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_manifest_v1.json"
PRE_TRUTH = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_gate_pre_truth_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase141_telemetry_schema_raw_result_v1.json"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase142_phase141_accuracy_result_v1.json"

PHASE137_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase137_phase136_accuracy.py"
PHASE134_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_accuracy.py"
PHASE118_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_accuracy.py"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase142_phase141_accuracy.py"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
STRUCTURAL_LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {ROUTES[0]: 2159, ROUTES[1]: 1466}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
STRICT_PROMOTION_THRESHOLD_M = 0.782

AUDIT_COMMIT = "dfca85dfabbbaf950cecbe0543802860e2279380"
AUDIT_SHA256 = "3b55ca5c7a9dcf8fbd5d1658e26d31a63bbe48cf6ce060951f62895840a2b276"
FREEZE_COMMIT = "3ab1429a431cd65ceca193dd997b397b0879aa9"
FREEZE_SHA256 = "dc2af8b53f7aca2b22cb0193231ab62304095fd832d18647b5686a73ea1f818b"
STRUCTURAL_COMMIT = "9b2427bb03ca14df5bf3c3668d46d3638b03388a"
STRUCTURAL_SHA256 = "33d67248b9423a841ebb29b6373eed042d4da6b15924df710dcc37f18b73ff13"
PHASE137_EVALUATOR_COMMIT = "97bc75d4e94b339507864a7b7c9f188ae33915fd"
PHASE137_EVALUATOR_SHA256 = "f294aecadca1e427673ada124f24f91bb5cb2b880156d8337fde8f7afc32c200"
PHASE134_EVALUATOR_COMMIT = "b59ca771dabcc1185910d3d95259572f76b16869"
PHASE134_EVALUATOR_SHA256 = "b554e2c07bedffcb53751e4e6351bcf5006fe99370dee34a16cc4f3eb81b145a"
PHASE118_EVALUATOR_COMMIT = "c29e1700922260a77f047db8c82692f9d2b13603"
PHASE118_EVALUATOR_SHA256 = "a1b2fac0a5615e47a11e7e4b171ced77d12818b66c598a531a4c48d2bf7ff829"
PHASE82_EVALUATOR_COMMIT = "be45879297a2a00a8574c77b71be0a5b721ff8c9"
PHASE82_EVALUATOR_SHA256 = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE76_PARSER_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
PHASE76_PARSER_SHA256 = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
PHASE112_COMMIT = "0151bd070f8029413f8f2a17a28fe982dddb735f"
PHASE112_RESULT_SHA256 = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"

RESULT_SCHEMA = "smartphone-r5-phase142-phase141-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase142-phase141-accuracy-gate-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase142-phase141-accuracy-truth-authorization.v1"
CANDIDATE_ID = "phase142-phase141-telemetry-schema-opaque-truth-only-accuracy-v1"


class Phase142AccuracyError(ValueError):
    """A contract violation; all verification and evaluation failures close."""


def fail(message: str) -> Phase142AccuracyError:
    return Phase142AccuracyError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value: Any, label: str) -> None:
    if value is not True:
        raise fail(f"{label}: expected true, got {value!r}")


def finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _payload_basename(name: str) -> bool:
    lowered = name.lower()
    return lowered in {
        "opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv",
        "device_imu.csv", "brdc.nav", "base.obs", "truth.csv",
    } or lowered.endswith(".mat")


def sha256_static(path: Path, label: str) -> str:
    """Hash tracked/static artifacts only; never probe a payload in this lane."""
    if _payload_basename(path.name):
        raise fail(f"payload hash forbidden in launch-free mode: {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    return digest.hexdigest()


def _pin(container: Mapping[str, Any], key: str, path: Path, expected: str, label: str | None = None) -> None:
    item = container.get(key)
    name = label or key
    if not isinstance(item, Mapping):
        raise fail(f"missing authority pin: {name}")
    assert_equal(item.get("path"), relative(path), f"authority/{name}/path")
    assert_equal(item.get("sha256"), expected, f"authority/{name}/sha256")


def _deferred_path(value: Any, basename: str, label: str, fragment: str) -> str:
    if not isinstance(value, str) or not value:
        raise fail(f"missing {label} path metadata")
    path = Path(value)
    if path.is_absolute() or ".." in path.parts or path.name != basename or fragment not in value:
        raise fail(f"unsafe deferred {label} path: {value}")
    return value


def _load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load pinned source: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def metric_contract() -> dict[str, Any]:
    """Return the exact Phase134 metric contract after Phase137 parity check."""
    if sha256_static(PHASE137_EVALUATOR, "Phase137 metric evaluator") != PHASE137_EVALUATOR_SHA256:
        raise fail("Phase137 metric evaluator hash changed")
    if sha256_static(PHASE134_EVALUATOR, "Phase134 metric evaluator") != PHASE134_EVALUATOR_SHA256:
        raise fail("Phase134 metric evaluator hash changed")
    phase137 = _load_module(PHASE137_EVALUATOR, "phase137_metric_reference")
    phase134 = _load_module(PHASE134_EVALUATOR, "phase134_metric_reference")
    contract_137 = phase137.metric_contract()
    contract_134 = phase134.metric_contract()
    assert_equal(contract_137, contract_134, "Phase137/Phase134 metric contract parity")
    return contract_134


def _verify_metric_contract(value: Any, label: str) -> None:
    assert_equal(value, metric_contract(), label)


def _verify_zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label} missing")
    for key in (
        "candidate_paths_materialized", "truth_paths_materialized", "candidate_solution_payload_reads",
        "candidate_coordinate_interpretations", "truth_reads", "accuracy_calculations",
        "native_solver_invocations", "raw_gnss_imu_navigation_reads", "raw_base_rinex_reads",
        "mat_pdc_precomputed_coordinate_reads", "kaggle_or_token_access", "reruns", "fallbacks", "repairs",
    ):
        assert_equal(value.get(key), 0, f"{label}/{key}")
    for key in ("raw_content_copied_or_transformed", "solution_rows_in_result", "solution_output_published"):
        assert_equal(value.get(key), False, f"{label}/{key}")


def _verify_policy(policy: Any, label: str) -> None:
    if not isinstance(policy, Mapping):
        raise fail(f"{label} missing")
    for key in (
        "accuracy_evaluation", "fallback", "kaggle_access", "mat_used", "pdc_input_used",
        "precomputed_coordinates_used", "repair", "rerun", "solution_coordinate_interpretation",
        "solution_publication", "truth_used",
    ):
        assert_equal(policy.get(key), False, f"{label}/{key}")


def _verify_route_metrics(route: str, item: Mapping[str, Any]) -> None:
    assert_equal(item.get("status"), "native-returned", f"structural/{route}/status")
    assert_equal(item.get("structural_gate"), "pass", f"structural/{route}/gate")
    assert_equal(item.get("solver_invocations"), 1, f"structural/{route}/solver invocations")
    assert_equal(item.get("solution_content_read"), False, f"structural/{route}/solution read")
    opaque = item.get("opaque_solution")
    if not isinstance(opaque, Mapping):
        raise fail(f"structural/{route}/opaque solution missing")
    _deferred_path(opaque.get("path"), "opaque_solution_output.csv", f"structural {route} solution", "/phase141-telemetry-schema-structural-v1/")
    assert_true(isinstance(opaque.get("sha256"), str) and len(opaque["sha256"]) == 64, f"structural/{route}/opaque hash")
    assert_true(isinstance(opaque.get("bytes"), int) and opaque["bytes"] > 0, f"structural/{route}/opaque bytes")
    assert_true(isinstance(opaque.get("newline_count"), int) and opaque["newline_count"] > 0, f"structural/{route}/opaque newlines")
    assert_equal(opaque.get("content_interpreted"), False, f"structural/{route}/opaque interpretation")
    native = item.get("native_summary")
    if not isinstance(native, Mapping):
        raise fail(f"structural/{route}/native summary missing")
    _deferred_path(native.get("path"), "native_summary.json", f"structural {route} native summary", "/phase141-telemetry-schema-structural-v1/")
    assert_equal(native.get("coordinate_fields_interpreted"), False, f"structural/{route}/native coordinate interpretation")
    counts = item.get("input_read_counts")
    assert_equal(counts, {"base.obs": 1, "brdc.nav": 1, "device_gnss.csv": 1, "device_imu.csv": 1}, f"structural/{route}/input reads")
    authority = item.get("authority_metrics")
    if not isinstance(authority, Mapping):
        raise fail(f"structural/{route}/authority metrics missing")
    factors = authority.get("factor_counts")
    if not isinstance(factors, Mapping):
        raise fail(f"structural/{route}/factor counts missing")
    for key in ("pseudorange", "doppler", "ordinary_tdcp"):
        value = factors.get(key)
        assert_true(isinstance(value, int) and value > 0, f"structural/{route}/factor/{key}")
    for key in ("legacy_pseudorange", "legacy_doppler", "legacy_tdcp"):
        assert_equal(factors.get(key), 0, f"structural/{route}/factor/{key}")
    phase135 = authority.get("phase135")
    if not isinstance(phase135, Mapping):
        raise fail(f"structural/{route}/Phase135 metrics missing")
    for family, factor_key in (("pseudorange", "pseudorange"), ("doppler", "doppler"), ("ordinary_tdcp", "ordinary_tdcp")):
        family_item = phase135.get(family)
        if not isinstance(family_item, Mapping):
            raise fail(f"structural/{route}/Phase135/{family} missing")
        assert_equal(family_item.get("admitted_rows"), family_item.get("affine_factors_inserted"), f"structural/{route}/Phase135/{family}/admitted-affine")
        assert_equal(family_item.get("affine_factors_inserted"), factors[factor_key], f"structural/{route}/Phase135/{family}/factor count")
    legacy = phase135.get("legacy_factor_counts")
    if not isinstance(legacy, Mapping):
        raise fail(f"structural/{route}/Phase135/legacy counts missing")
    for key in ("pseudorange", "doppler", "ordinary_tdcp"):
        assert_equal(legacy.get(key), 0, f"structural/{route}/Phase135/legacy/{key}")
    bridge = authority.get("bridge")
    assert_equal(bridge.get("pose3_x_keys_exact") if isinstance(bridge, Mapping) else None, True, f"structural/{route}/bridge keys")
    clock = authority.get("clock")
    if not isinstance(clock, Mapping):
        raise fail(f"structural/{route}/clock missing")
    assert_equal(clock.get("c_units"), "metres", f"structural/{route}/C units")
    assert_equal(clock.get("d_units"), "metres/second", f"structural/{route}/D units")
    assert_equal(clock.get("c_epoch_count"), clock.get("d_epoch_count"), f"structural/{route}/C/D epochs")
    assert_equal(clock.get("c_finite_count"), clock.get("c_epoch_count"), f"structural/{route}/C finite")
    assert_equal(clock.get("d_finite_count"), clock.get("d_epoch_count"), f"structural/{route}/D finite")
    phase138 = authority.get("phase138")
    if not isinstance(phase138, Mapping):
        raise fail(f"structural/{route}/Phase138 missing")
    assert_equal(phase138.get("adjusted_exactly_once"), True, f"structural/{route}/Phase138 exactly once")
    assert_equal(phase138.get("factor_count_unchanged"), True, f"structural/{route}/Phase138 count invariant")
    assert_equal(phase138.get("affine_tdcp_factor_count"), factors["ordinary_tdcp"], f"structural/{route}/Phase138 factor count")
    assert_equal(phase138.get("tdcp_measurements_adjusted"), factors["ordinary_tdcp"], f"structural/{route}/Phase138 adjustment count")
    assert_equal(phase138.get("adjustment_application_passes"), 1, f"structural/{route}/Phase138 pass count")
    raw_base = authority.get("raw_base")
    if not isinstance(raw_base, Mapping):
        raise fail(f"structural/{route}/raw base missing")
    assert_equal(raw_base.get("applied_exactly_once"), True, f"structural/{route}/raw base exactly once")
    assert_equal(raw_base.get("correction_application_pass_count"), 1, f"structural/{route}/raw base pass count")
    assert_equal(raw_base.get("corrected_rows"), factors["pseudorange"], f"structural/{route}/raw base rows")
    assert_equal(raw_base.get("no_raw_or_zero_fallback"), True, f"structural/{route}/raw base fallback")
    solver = authority.get("solver")
    if not isinstance(solver, Mapping):
        raise fail(f"structural/{route}/solver missing")
    assert_equal(solver.get("linear_solver"), "MULTIFRONTAL_QR", f"structural/{route}/solver")
    assert_equal(solver.get("elimination"), "EliminateQR", f"structural/{route}/elimination")
    for stage_name in ("gnss_first", "main"):
        stage = solver.get(stage_name)
        if not isinstance(stage, Mapping):
            raise fail(f"structural/{route}/{stage_name} missing")
        assert_equal(stage.get("attempted"), True, f"structural/{route}/{stage_name}/attempted")
        assert_true(isinstance(stage.get("accepted_iterations"), int) and stage["accepted_iterations"] > 0, f"structural/{route}/{stage_name}/accepted")
        assert_true(finite(stage.get("initial_cost")) and finite(stage.get("final_cost")), f"structural/{route}/{stage_name}/cost finite")
        assert_true(stage["final_cost"] < stage["initial_cost"], f"structural/{route}/{stage_name}/cost decrease")
        assert_equal(stage.get("costs_finite"), True, f"structural/{route}/{stage_name}/finite")
        assert_equal(stage.get("strict_cost_decrease"), True, f"structural/{route}/{stage_name}/strict decrease")
        assert_equal(stage.get("no_fallback"), True, f"structural/{route}/{stage_name}/fallback")
    output = authority.get("output")
    if not isinstance(output, Mapping):
        raise fail(f"structural/{route}/output missing")
    for key in ("coordinate_rows_interpreted",):
        assert_equal(output.get(key), False, f"structural/{route}/output/{key}")
    for key in ("finite", "earth_valid", "expected_epoch_coverage", "opaque_solution_seal"):
        assert_equal(output.get(key), True, f"structural/{route}/output/{key}")
    assert_equal(output.get("pixel5_offset_applications"), 1, f"structural/{route}/Pixel5 offset")


def verify_structural_result() -> dict[str, Any]:
    assert_equal(sha256_static(STRUCTURAL_RESULT, "Phase141 structural result"), STRUCTURAL_SHA256, "structural/sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase141 structural result")
    for key, expected in {
        "schema_version": "smartphone-r5-phase141-telemetry-schema-raw-result.v1",
        "phase": 141,
        "status": "sealed-structural-raw-result-no-accuracy",
        "structural_gate": "GO",
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
    }.items():
        assert_equal(result.get(key), expected, f"structural/{key}")
    _verify_policy(result.get("policy"), "structural/policy")
    accounting = result.get("read_accounting")
    if not isinstance(accounting, Mapping):
        raise fail("structural/read_accounting missing")
    for key, expected in {
        "raw_phone_gnss_reads": 2, "raw_phone_imu_reads": 2,
        "broadcast_navigation_reads": 2, "raw_base_rinex_reads": 2,
        "native_solver_invocations": 2, "truth_reads": 0,
        "accuracy_calculations": 0, "solution_coordinate_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0,
        "reruns_fallbacks_repairs_sweeps": 0,
    }.items():
        assert_equal(accounting.get(key), expected, f"structural/read_accounting/{key}")
    routes = result.get("routes")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("structural/routes missing or not a two-route list")
    by_route: dict[str, Mapping[str, Any]] = {}
    for item in routes:
        if not isinstance(item, Mapping) or item.get("route") not in ROUTES:
            raise fail("structural route identity malformed")
        by_route[item["route"]] = item
    assert_equal(list(by_route), list(ROUTES), "structural/route order")
    expected_opaque = {
        ROUTES[0]: ("e1f8211b16a413622c95903e73d5b5f17fff0ee9acdb28bdce5bdd15708b229c", 172694, 2159),
        ROUTES[1]: ("85f5610f3deffae9912ed9ea026be2b4a4e0824064516e6c93e591255c1f4fcd", 117254, 1466),
    }
    for route in ROUTES:
        _verify_route_metrics(route, by_route[route])
        opaque = by_route[route]["opaque_solution"]
        digest, bytes_count, newline_count = expected_opaque[route]
        assert_equal(opaque.get("sha256"), digest, f"structural/{route}/opaque hash")
        assert_equal(opaque.get("bytes"), bytes_count, f"structural/{route}/opaque bytes")
        assert_equal(opaque.get("newline_count"), newline_count, f"structural/{route}/opaque newlines")
    return result


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase142 audit"), AUDIT_SHA256, "audit/sha256")
    freeze = read_json(FREEZE, "Phase142 accuracy freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase142-phase141-accuracy-gate-freeze.v1",
        "phase": 142,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase142-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("freeze/authority missing")
    _pin(authority, "audit", AUDIT, AUDIT_SHA256)
    assert_equal(authority["audit"].get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    _pin(authority, "phase141_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256)
    assert_equal(authority["phase141_structural_result"].get("commit"), STRUCTURAL_COMMIT, "freeze/structural/commit")
    source_pins = {
        "phase137_metric_contract": (PHASE137_EVALUATOR, PHASE137_EVALUATOR_SHA256, PHASE137_EVALUATOR_COMMIT),
        "phase134_metric_evaluator": (PHASE134_EVALUATOR, PHASE134_EVALUATOR_SHA256, PHASE134_EVALUATOR_COMMIT),
        "phase118_metric_evaluator": (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256, PHASE118_EVALUATOR_COMMIT),
        "phase82_scorer": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, PHASE82_EVALUATOR_COMMIT),
        "phase76_truth_parser": (PHASE76_PARSER, PHASE76_PARSER_SHA256, PHASE76_PARSER_COMMIT),
        "phase112_baseline": (ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", PHASE112_RESULT_SHA256, PHASE112_COMMIT),
    }
    for key, (path, digest, commit) in source_pins.items():
        _pin(authority, key, path, digest)
        assert_equal(authority[key].get("commit"), commit, f"freeze/{key}/commit")
        assert_equal(sha256_static(path, f"freeze/{key}"), digest, f"freeze/{key}/sha256")
    structural = verify_structural_result()
    candidate = freeze.get("candidate")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "id": CANDIDATE_ID, "candidate_count": 1, "route_order_fixed": True,
        "runs_per_route": 1, "solver_rerun": False, "raw_or_base_rerun": False,
        "candidate_files_copied_or_transformed": False, "solution_repaired_or_replaced": False,
        "solution_publication": False, "coordinate_interpretation_before_truth": False,
        "metadata_hash_seal_precedes_truth": True,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("route_order"), list(ROUTES), "freeze/candidate/route order")
    assert_equal(candidate.get("phase141_pixel5_offset_boundary", {}).get("already_applied_in_structural_run"), True, "freeze/offset/applied")
    assert_equal(candidate.get("phase141_pixel5_offset_boundary", {}).get("evaluator_must_not_apply_again"), True, "freeze/offset/reapply")
    assert_equal(candidate.get("phase141_pixel5_offset_boundary", {}).get("application_count_per_route"), 1, "freeze/offset/count")
    metadata = candidate.get("routes_metadata")
    if not isinstance(metadata, Mapping):
        raise fail("freeze/candidate/routes_metadata missing")
    structural_routes = {item["route"]: item for item in structural["routes"]}
    for route in ROUTES:
        item = metadata.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"freeze candidate metadata missing: {route}")
        _deferred_path(item.get("path"), "opaque_solution_output.csv", f"candidate {route}", "/phase141-telemetry-schema-structural-v1/")
        opaque = structural_routes[route]["opaque_solution"]
        for key, expected in {
            "label": STRUCTURAL_LABELS[route], "sha256": opaque["sha256"],
            "bytes": opaque["bytes"], "newline_count": opaque["newline_count"],
            "expected_prediction_rows": DOMAIN_ROWS[route],
            "expected_problem_epochs": PROBLEM_EPOCHS[route],
            "opaque_metadata_seal": True, "path_materialization_before_authorization": False,
        }.items():
            assert_equal(item.get(key), expected, f"freeze/candidate/{route}/{key}")
    truth = freeze.get("truth_cohort")
    if not isinstance(truth, Mapping):
        raise fail("freeze/truth_cohort missing")
    for key, expected in {
        "route_order": list(ROUTES), "read_by_solver": False, "read_by_audit": False,
        "read_by_freeze": False, "read_by_manifest": False, "read_by_evaluator_only": True,
    }.items():
        assert_equal(truth.get(key), expected, f"freeze/truth/{key}")
    truth_routes = truth.get("routes")
    if not isinstance(truth_routes, Mapping):
        raise fail("freeze/truth/routes missing")
    for route in ROUTES:
        item = truth_routes.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"freeze truth metadata missing: {route}")
        _deferred_path(item.get("path"), "ground_truth.csv", f"truth {route}", "/truth/")
        assert_equal(item.get("rows"), TRUTH_ROWS[route], f"freeze/truth/{route}/rows")
        assert_true(isinstance(item.get("sha256"), str) and len(item["sha256"]) == 64, f"freeze/truth/{route}/hash")
    _verify_metric_contract(freeze.get("metric_contract"), "freeze/metric_contract")
    baselines = freeze.get("comparison_baselines")
    if not isinstance(baselines, Mapping):
        raise fail("freeze/comparison_baselines missing")
    assert_equal(baselines.get("phase112_macro_m"), 0.8318381724, "freeze/Phase112/macro")
    assert_equal(baselines.get("phase118_champion_macro_m"), 0.8141981503, "freeze/Phase118/macro")
    assert_equal(baselines.get("informational_only"), True, "freeze/baselines/informational")
    gates = freeze.get("promotion_gates")
    if not isinstance(gates, Mapping):
        raise fail("freeze/promotion_gates missing")
    for key, expected in {
        "all_gates_anded": True, "exact_two_routes_one_evaluation_each": True,
        "candidate_output_schema_and_exact_alignment": True,
        "candidate_prediction_domain_coverage_exact": 1.0,
        "candidate_all_finite_and_earth_valid": True, "candidate_over_70_mps_count_zero": True,
        "pixel5_offset_already_exactly_once_no_reapplication": True,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": 0.782, "equality_at_threshold_passes": False,
        "truth_read_only_by_one_evaluator_process": True, "solution_rows_absent_from_result": True,
        "no_solver_raw_base_mat_pdc_precomputed_or_kaggle": True,
    }.items():
        assert_equal(gates.get(key), expected, f"freeze/promotion_gates/{key}")
    boundary = freeze.get("truth_evaluator_boundary")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/truth_evaluator_boundary missing")
    for key, expected in {
        "authorization_required_before_candidate_or_truth_path_materialization": True,
        "candidate_metadata_hash_seal_precedes_truth": True, "candidate_parse_precedes_truth": True,
        "evaluator_process_count": 1, "candidate_solution_reads_for_hash_and_parse": 2,
        "candidate_coordinate_interpretations": 2, "truth_reads": 2,
        "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "truth_path_or_bytes_in_native": False,
        "native_after_truth": False, "no_solution_rows_in_result": True,
        "no_truth_coordinate_rows_in_result": True, "release_or_submission": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/boundary/{key}")
    accounting = freeze.get("read_accounting")
    if not isinstance(accounting, Mapping):
        raise fail("freeze/read_accounting missing")
    _verify_zero_accounting(accounting.get("audit_before_freeze"), "freeze/audit_before_freeze")
    planned = accounting.get("planned_truth_evaluation")
    if not isinstance(planned, Mapping):
        raise fail("freeze/planned_truth_evaluation missing")
    for key, expected in {
        "candidate_paths_materialized": 2, "truth_paths_materialized": 2,
        "candidate_solution_reads_for_hash_and_parse": 2, "candidate_coordinate_interpretations": 2,
        "truth_reads": 2, "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0,
    }.items():
        assert_equal(planned.get(key), expected, f"freeze/planned/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase142 accuracy manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA, "phase": 142, "execution_label": "Luna Max",
        "status": "launch-free-before-phase142-truth-only-authorization",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/routes")
    _verify_metric_contract(manifest.get("metric_contract"), "manifest/metric_contract")
    candidate_reference = manifest.get("candidate_reference")
    if not isinstance(candidate_reference, Mapping):
        raise fail("manifest/candidate_reference missing")
    for key, expected in {
        "source": "freeze.candidate", "candidate_count": 1,
        "route_order": list(ROUTES), "opaque_only": True,
        "solution_coordinate_rows_read_before_authorization": 0,
    }.items():
        assert_equal(candidate_reference.get(key), expected, f"manifest/candidate_reference/{key}")
    truth_reference = manifest.get("truth_reference")
    if not isinstance(truth_reference, Mapping):
        raise fail("manifest/truth_reference missing")
    for key, expected in {
        "source": "freeze.truth_cohort", "route_order": list(ROUTES),
        "truth_payload_reads_before_authorization": 0,
        "truth_payload_materialization_before_authorization": 0,
    }.items():
        assert_equal(truth_reference.get(key), expected, f"manifest/truth_reference/{key}")
    manifest_baselines = manifest.get("comparison_baselines")
    if not isinstance(manifest_baselines, Mapping):
        raise fail("manifest/comparison_baselines missing")
    for key, expected in {
        "phase112_macro_m": 0.8318381724,
        "phase118_champion_macro_m": 0.8141981503,
        "informational_only": True,
        "no_baseline_payload_read_or_rerun": True,
    }.items():
        assert_equal(manifest_baselines.get(key), expected, f"manifest/baselines/{key}")
    expected_freeze = {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": FREEZE_SHA256}
    expected_structural = {"path": relative(STRUCTURAL_RESULT), "commit": STRUCTURAL_COMMIT, "sha256": STRUCTURAL_SHA256}
    assert_equal(manifest.get("freeze"), expected_freeze, "manifest/freeze")
    assert_equal(manifest.get("structural_result"), expected_structural, "manifest/structural_result")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest/authority missing")
    for key, path, expected in (
        ("audit", AUDIT, AUDIT_SHA256), ("accuracy_freeze", FREEZE, FREEZE_SHA256),
        ("phase141_structural_result", STRUCTURAL_RESULT, STRUCTURAL_SHA256),
        ("phase137_metric_contract", PHASE137_EVALUATOR, PHASE137_EVALUATOR_SHA256),
        ("phase134_metric_evaluator", PHASE134_EVALUATOR, PHASE134_EVALUATOR_SHA256),
        ("phase118_metric_evaluator", PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256),
        ("phase82_scorer", PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256),
        ("phase76_truth_parser", PHASE76_PARSER, PHASE76_PARSER_SHA256),
    ):
        _pin(authority, key, path, expected)
        assert_equal(sha256_static(path, f"manifest/{key}"), expected, f"manifest/{key}/sha256")
    for key, path in (("evaluator", EVALUATOR), ("focused_tests", FOCUSED_TESTS)):
        item = authority.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest/{key} pin missing")
        assert_equal(item.get("path"), relative(path), f"manifest/{key}/path")
        digest = item.get("sha256")
        assert_true(isinstance(digest, str) and len(digest) == 64, f"manifest/{key}/hash format")
        assert_equal(sha256_static(path, f"manifest/{key}"), digest, f"manifest/{key}/sha256")
    _verify_zero_accounting(manifest.get("read_accounting_before_authorization"), "manifest/read_accounting_before_authorization")
    requirements = manifest.get("authorization_requirements")
    if not isinstance(requirements, Mapping):
        raise fail("manifest/authorization_requirements missing")
    for key, expected in {
        "independent_commit_required": True, "candidate_and_truth_paths_materialized_only_after_auth": True,
        "route_order": list(ROUTES), "exact_candidate_reads": 2, "exact_truth_reads": 2,
        "exact_accuracy_calculations": 2, "native_solver_invocations": 0, "raw_reads": 0,
        "base_reads": 0, "reruns": 0, "fallbacks": 0, "repairs": 0,
        "solution_publication": False, "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0, "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": 0.782,
    }.items():
        assert_equal(requirements.get(key), expected, f"manifest/authorization/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    manifest = verify_manifest()
    pre = read_json(PRE_TRUTH, "Phase142 pre-truth seal")
    for key, expected in {
        "schema_version": "smartphone-r5-phase142-phase141-accuracy-gate-pre-truth.v1",
        "phase": 142, "status": "launch-free-pre-truth-sealed", "execution_label": "Luna Max",
    }.items():
        assert_equal(pre.get(key), expected, f"pre_truth/{key}")
    assert_equal(pre.get("manifest", {}).get("sha256"), sha256_static(MANIFEST, "pre-truth manifest"), "pre_truth/manifest sha")
    assert_equal(pre.get("freeze", {}).get("sha256"), FREEZE_SHA256, "pre_truth/freeze sha")
    assert_equal(pre.get("structural_result", {}).get("sha256"), STRUCTURAL_SHA256, "pre_truth/structural sha")
    assert_equal(pre.get("candidate", {}).get("candidate_count"), manifest["candidate_reference"]["candidate_count"], "pre_truth/candidate count")
    _verify_zero_accounting(pre.get("read_accounting"), "pre_truth/read_accounting")
    for key, expected in {
        "candidate_paths_materialized": 0, "truth_paths_materialized": 0,
        "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0,
        "truth_reads": 0, "accuracy_calculations": 0,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0, "repairs": 0,
        "truth_only_authorized": False, "raw_execution_or_solver_authorized": False,
        "solution_output_published": False,
    }.items():
        assert_equal(pre.get("read_accounting", {}).get(key), expected, f"pre_truth/read_accounting/{key}")
    return pre


def verify_authorization() -> dict[str, Any]:
    """Verify future independent authorization without opening payloads."""
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase142 truth-only authorization")
    for key, expected in {
        "schema_version": AUTHORIZATION_SCHEMA, "phase": 142,
        "execution_label": "Luna Max", "status": "authorized-for-phase142-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
    assert_equal(auth.get("manifest_sha256"), sha256_static(MANIFEST, "authorization manifest"), "authorization/manifest sha")
    pins = auth.get("authority")
    if not isinstance(pins, Mapping):
        raise fail("authorization/authority missing")
    for key, path in (("audit", AUDIT), ("accuracy_freeze", FREEZE), ("accuracy_manifest", MANIFEST),
                      ("phase141_structural_result", STRUCTURAL_RESULT), ("accuracy_evaluator", EVALUATOR),
                      ("focused_tests", FOCUSED_TESTS), ("phase137_metric_contract", PHASE137_EVALUATOR),
                      ("phase134_metric_evaluator", PHASE134_EVALUATOR), ("phase118_metric_evaluator", PHASE118_EVALUATOR),
                      ("phase82_scorer", PHASE82_EVALUATOR), ("phase76_truth_parser", PHASE76_PARSER)):
        item = pins.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"authorization/{key} missing")
        digest = item.get("sha256")
        assert_true(isinstance(digest, str) and len(digest) == 64, f"authorization/{key}/hash")
        assert_equal(sha256_static(path, f"authorization/{key}"), digest, f"authorization/{key}/sha")
    policy = auth.get("execution_policy")
    if not isinstance(policy, Mapping):
        raise fail("authorization/execution_policy missing")
    for key, expected in {
        "truth_only_evaluation_authorized": True, "native_solver_invocations": 0,
        "raw_reads": 0, "base_reads": 0, "candidate_reads_per_route": 1,
        "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_after_truth": False, "solution_publication": False,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": STRICT_PROMOTION_THRESHOLD_M,
        "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0, "repairs": 0,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    return auth


def materialize_candidate_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("candidate path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = freeze["candidate"]["routes_metadata"][route]["path"]
    _deferred_path(text, "opaque_solution_output.csv", f"candidate {route}", "/phase141-telemetry-schema-structural-v1/")
    return ROOT / text


def materialize_truth_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("truth path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = freeze["truth_cohort"]["routes"][route]["path"]
    _deferred_path(text, "ground_truth.csv", f"truth {route}", "/truth/")
    return ROOT / text


def load_phase82() -> Any:
    if sha256_static(PHASE82_EVALUATOR, "Phase82 scorer") != PHASE82_EVALUATOR_SHA256:
        raise fail("Phase82 scorer hash changed")
    return _load_module(PHASE82_EVALUATOR, "phase82_accuracy_reference_phase142")


def read_candidate_once(path: Path, seal: Mapping[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256") or len(payload) != seal.get("bytes"):
        raise fail(f"candidate opaque seal mismatch: {route}")
    ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered):
        raise fail(f"candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {
        "path": relative(path), "bytes": len(payload), "sha256": digest,
        "header": "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees",
        "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True,
    }


def read_truth_once(path: Path, pin: Mapping[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail(f"truth opaque seal mismatch: {route}")
    truth = p82.P76._parse_truth_dictreader(payload, route)
    if len(truth) != pin.get("rows"):
        raise fail(f"truth row count mismatch: {route}")
    return truth, {"path": relative(path), "bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < STRICT_PROMOTION_THRESHOLD_M


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    """Future truth-only lane; exactly two payload reads after authorization."""
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_freeze()
    p82 = load_phase82()
    accounting = {"candidate_paths_materialized": 0, "truth_paths_materialized": 0, "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0, "truth_reads": 0, "accuracy_calculations": 0}
    reports: dict[str, Any] = {}
    for route in ROUTES:
        candidate_path = materialize_candidate_path(freeze, route, authorized=True)
        accounting["candidate_paths_materialized"] += 1
        ordered, candidate, candidate_meta = read_candidate_once(candidate_path, freeze["candidate"]["routes_metadata"][route], route, p82)
        accounting["candidate_solution_reads"] += 1
        accounting["candidate_coordinate_interpretations"] += 1
        truth_path = materialize_truth_path(freeze, route, authorized=True)
        accounting["truth_paths_materialized"] += 1
        truth, truth_meta = read_truth_once(truth_path, freeze["truth_cohort"]["routes"][route], route, p82)
        accounting["truth_reads"] += 1
        score = p82.P76._score_prediction(candidate, truth, freeze["truth_cohort"]["routes"][route].get("expected_missing_truth_key"), route, ordered)
        accounting["accuracy_calculations"] += 1
        checks = {
            "candidate_schema_and_exact_alignment": len(ordered) == DOMAIN_ROWS[route],
            "candidate_prediction_domain_coverage_exact": score.get("prediction_domain_coverage") == 1.0,
            "candidate_all_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_finite": finite(score.get("score_m")),
        }
        reports[route] = {"dataset_id": route, "candidate_solution": candidate_meta, "truth": truth_meta, "candidate": score, "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]}}
    candidate_macro = sum(float(reports[route]["candidate"]["score_m"]) for route in ROUTES) / 2.0
    gates = {
        "exact_two_routes_one_evaluation_each": accounting == {"candidate_paths_materialized": 2, "truth_paths_materialized": 2, "candidate_solution_reads": 2, "candidate_coordinate_interpretations": 2, "truth_reads": 2, "accuracy_calculations": 2},
        "candidate_output_schema_and_exact_alignment": all(reports[route]["gates"]["checks"]["candidate_schema_and_exact_alignment"] for route in ROUTES),
        "candidate_prediction_domain_coverage_exact": all(reports[route]["candidate"].get("prediction_domain_coverage") == 1.0 for route in ROUTES),
        "candidate_all_finite_and_earth_valid": all(reports[route]["candidate"].get("finite") is True for route in ROUTES),
        "candidate_over_70_mps_count_zero": all(reports[route]["candidate"].get("over_70_mps_count") == 0 for route in ROUTES),
        "candidate_route_scores_finite": all(finite(reports[route]["candidate"].get("score_m")) for route in ROUTES),
        "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(candidate_macro),
        "truth_read_only_by_one_evaluator_process": accounting["truth_reads"] == 2,
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_mat_pdc_precomputed_or_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA, "phase": 142, "execution_label": "Luna Max",
        "status": "go-phase142-truth-only-accuracy" if not failed else "no-go-phase142-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID, "accuracy_scored": True, "routes": reports,
        "aggregate": {"candidate_macro_score_m": candidate_macro, "phase112_macro_m": manifest["comparison_baselines"]["phase112_macro_m"], "phase118_champion_macro_m": manifest["comparison_baselines"]["phase118_champion_macro_m"], "route_count": 2, "macro_route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"},
        "metric_contract": manifest["metric_contract"], "promotion_gates": gates,
        "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": STRICT_PROMOTION_THRESHOLD_M, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]},
        "failed_gates": failed, "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"], "candidate_coordinate_interpretations": accounting["candidate_coordinate_interpretations"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "accuracy_calculations": accounting["accuracy_calculations"], "candidate_paths_materialized_after_authorization": accounting["candidate_paths_materialized"], "truth_paths_materialized_after_authorization": accounting["truth_paths_materialized"], "reruns": 0, "fallbacks": 0, "repairs": 0, "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0},
        "solution_output_published": False, "release_or_submission_authorized": False,
    }
    result_path.parent.mkdir(parents=True, exist_ok=True)
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
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--verify-authorization", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    modes = [args.verify_freeze, args.verify_manifest, args.verify_pre_truth, args.verify_authorization, args.evaluate]
    if sum(bool(mode) for mode in modes) != 1:
        parser.error("choose exactly one verification/evaluation mode")
    try:
        if args.verify_freeze:
            verify_freeze()
        elif args.verify_manifest:
            verify_manifest()
        elif args.verify_pre_truth:
            print(json.dumps(verify_pre_truth(), indent=2, sort_keys=True))
        elif args.verify_authorization:
            verify_authorization()
        else:
            result = evaluate(args.result_json)
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0 if result.get("status", "").startswith("go-") else 1
        return 0
    except Exception as exc:
        print(f"phase142 truth-only evaluator: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
