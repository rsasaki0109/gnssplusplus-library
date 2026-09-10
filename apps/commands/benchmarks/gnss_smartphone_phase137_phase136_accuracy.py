#!/usr/bin/env python3
"""Launch-free contract and isolated truth-only evaluator for Phase137.

The verification modes read only tracked source and sealed metadata.  They do
not materialize or probe candidate solutions, official truth, raw inputs, or
native output, and they never launch a solver.  The optional evaluation mode
is intentionally gated by an independent truth-only authorization artifact;
only that mode may read one immutable candidate and one truth payload per
route.  Metric/parser/join/aggregation behavior is delegated to the pinned
Phase134/Phase118/Phase82/Phase76 implementation.
"""

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Mapping


ROOT = Path(__file__).resolve().parents[3]
EVALUATOR = Path(__file__).resolve()
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_gate_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_gate_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_gate_manifest_v1.json"
PRE_TRUTH = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_gate_pre_truth_v1.json"
AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_truth_authorization_v1.json"
STRUCTURAL_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase136_official_affine_structural_raw_result_v1.json"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_result_v1.json"

PHASE134_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase134_native_summary_bridge_accuracy.py"
PHASE118_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase118_tdcp_robust_k_accuracy.py"
PHASE82_EVALUATOR = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
PHASE76_PARSER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
DOMAIN_ROWS = {ROUTES[0]: 2158, ROUTES[1]: 1465}
PROBLEM_EPOCHS = {ROUTES[0]: 2159, ROUTES[1]: 1466}
TRUTH_ROWS = {ROUTES[0]: 2159, ROUTES[1]: 1465}
STRUCTURAL_LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
STRICT_PROMOTION_THRESHOLD_M = 0.782

AUDIT_COMMIT = "3f7cf4afcf08405b37c17c659749200c6265b496"
AUDIT_SHA256 = "e2d5c7e5d089c32af17c71e89a8ff6f9ac7b4e37165f760030e26774be9f548d"
FREEZE_COMMIT = "7eee7a293f87d82b4c0be9dbeae4b58b8ca79861"
STRUCTURAL_COMMIT = "65b24a939a373421015f07b119fdbec04d5c6ebb"
STRUCTURAL_SHA256 = "d79dbac9dba4ce352d1d2b23f92d87fa44057221822a67f94c494f41f789c2a7"
PHASE134_EVALUATOR_COMMIT = "b59ca771dabcc1185910d3d95259572f76b16869"
PHASE134_EVALUATOR_SHA256 = "b554e2c07bedffcb53751e4e6351bcf5006fe99370dee34a16cc4f3eb81b145a"
PHASE134_FREEZE_COMMIT = "75a3037114595ebe339fb0317ec50bfa45419cae"
PHASE134_FREEZE_SHA256 = "8aebc8157f94f2b12470a79e583b4632742e06f63322315d234a168eadcb2a87"
PHASE118_EVALUATOR_COMMIT = "c29e1700922260a77f047db8c82692f9d2b13603"
PHASE118_EVALUATOR_SHA256 = "a1b2fac0a5615e47a11e7e4b171ced77d12818b66c598a531a4c48d2bf7ff829"
PHASE118_FREEZE_COMMIT = "ef8d11bc4fa64e2ed6cd5fface133979655953ff"
PHASE118_FREEZE_SHA256 = "860a7f382199958f8dd2c43ba697d4c5cde1bc550d5747028e4c6f73be346760"
PHASE118_RESULT_SHA256 = "918597aab19f462a2aadfff606f461ce47f48f2fdcd997e5fdeb89ab459a0164"
PHASE82_EVALUATOR_COMMIT = "be45879297a2a00a8574c77b71be0a5b721ff8c9"
PHASE82_EVALUATOR_SHA256 = "232ba0ab1fa7ebde6ddfaaf851d190d1ed32377ea056ab6c31e244139c2977fb"
PHASE76_PARSER_COMMIT = "7a8e77ce03f6f4354cf2f541098a53628faa23d5"
PHASE76_PARSER_SHA256 = "513e5970ee58846963f077b9960944e5fbf5f3535e5be24ca6e72493785f7761"
PHASE112_COMMIT = "0151bd070f8029413f8f2a17a28fe982dddb735f"
PHASE112_RESULT_SHA256 = "46d8d836758ac1a88b95ec2c1a0c0080cedfb753c12411ed74e4c2501549c57e"

RESULT_SCHEMA = "smartphone-r5-phase137-phase136-accuracy-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase137-phase136-accuracy-gate-manifest.v1"
AUTHORIZATION_SCHEMA = "smartphone-r5-phase137-phase136-accuracy-truth-authorization.v1"
CANDIDATE_ID = "phase137-phase136-official-affine-opaque-truth-only-accuracy-v1"


class Phase137AccuracyError(ValueError):
    """A truth-only contract violation; all failures are closed."""


def fail(message: str) -> Phase137AccuracyError:
    return Phase137AccuracyError(message)


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


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(value, handle, indent=2, sort_keys=True, allow_nan=False)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except BaseException:
        try:
            os.unlink(temporary)
        except OSError:
            pass
        raise


def _payload_basename(name: str) -> bool:
    lowered = name.lower()
    return lowered in {
        "opaque_solution_output.csv", "ground_truth.csv", "device_gnss.csv",
        "device_imu.csv", "brdc.nav", "base.obs", "truth.csv",
    } or lowered.endswith((".mat",)) or "truth" in lowered


def sha256_static(path: Path, label: str) -> str:
    """Hash tracked/static artifacts only; never probe a payload pre-auth."""
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
    """Return the exact pinned Phase134 contract, without changing it."""
    if sha256_static(PHASE134_EVALUATOR, "Phase134 metric evaluator") != PHASE134_EVALUATOR_SHA256:
        raise fail("Phase134 metric evaluator hash changed")
    module = _load_module(PHASE134_EVALUATOR, "phase134_metric_reference")
    return module.metric_contract()


def _verify_metric_contract(value: Any, label: str) -> None:
    assert_equal(value, metric_contract(), label)


def _verify_zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, str):
            if item not in {"read-only", "sealed-metadata-only", "not-run"}:
                raise fail(f"{label}/{key} is not zero-read metadata")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_structural_result() -> dict[str, Any]:
    assert_equal(sha256_static(STRUCTURAL_RESULT, "Phase136 structural result"), STRUCTURAL_SHA256, "structural/sha256")
    result = read_json(STRUCTURAL_RESULT, "Phase136 structural result")
    for key, expected in {
        "schema_version": "smartphone-r5-phase136-official-affine-structural-raw-result.v1",
        "phase": 136,
        "recipe_phase": 135,
        "status": "sealed-structural-raw-result-no-accuracy",
    }.items():
        assert_equal(result.get(key), expected, f"structural/{key}")
    assert_equal(result.get("route_order"), [STRUCTURAL_LABELS[route] for route in ROUTES], "structural/route_order")
    assert_equal(result.get("runs_per_route"), 1, "structural/runs_per_route")
    accounting = result.get("read_accounting")
    if not isinstance(accounting, Mapping):
        raise fail("structural/read_accounting missing")
    for key, expected in {
        "raw_phone_gnss_reads": 2, "raw_phone_imu_reads": 2,
        "broadcast_navigation_reads": 2, "raw_base_rinex_reads": 2,
        "native_solver_invocations": 2, "truth_reads": 0,
        "solution_coordinate_reads": 0, "accuracy_calculations": 0,
        "mat_pdc_precomputed_coordinate_reads": 0, "kaggle_or_token_access": 0,
        "reruns_fallbacks_repairs_sweeps": 0,
    }.items():
        assert_equal(accounting.get(key), expected, f"structural/read_accounting/{key}")
    policy = result.get("policy")
    if not isinstance(policy, Mapping):
        raise fail("structural/policy missing")
    for key in (
        "truth_used", "mat_used", "pdc_used", "precomputed_coordinates_used",
        "accuracy_evaluation", "kaggle_access", "solution_publication",
        "solution_coordinate_interpretation", "fallback", "rerun", "repair",
    ):
        assert_equal(policy.get(key), False, f"structural/policy/{key}")
    routes = result.get("route_results")
    if not isinstance(routes, Mapping) or list(routes) != [STRUCTURAL_LABELS[route] for route in ROUTES]:
        raise fail("structural route set/order changed")
    for route in ROUTES:
        item = routes.get(STRUCTURAL_LABELS[route])
        if not isinstance(item, Mapping):
            raise fail(f"structural route missing: {route}")
        assert_equal(item.get("return_code"), 0, f"structural/{route}/return_code")
        assert_equal(item.get("solver_invocations"), 1, f"structural/{route}/solver_invocations")
        opaque = item.get("opaque_solution")
        if not isinstance(opaque, Mapping):
            raise fail(f"structural/{route}/opaque_solution missing")
        assert_true(isinstance(opaque.get("sha256"), str) and len(opaque["sha256"]) == 64, f"structural/{route}/opaque hash")
        assert_equal(opaque.get("content_interpreted"), False, f"structural/{route}/opaque/content_interpreted")
        assert_equal(opaque.get("newline_count"), PROBLEM_EPOCHS[route], f"structural/{route}/opaque/newline_count")
        for section in ("gnss_first", "main", "affine_families", "c7_d_and_units", "raw_base", "tdcp", "output"):
            if not isinstance(item.get(section), Mapping):
                raise fail(f"structural/{route}/{section} missing")
        stage = item["gnss_first"]
        main = item["main"]
        family = item["affine_families"]
        c7d = item["c7_d_and_units"]
        base = item["raw_base"]
        tdcp = item["tdcp"]
        output = item["output"]
        assert_equal(stage.get("epochs"), PROBLEM_EPOCHS[route], f"structural/{route}/stage/epochs")
        assert_true(stage.get("strict_cost_decrease"), f"structural/{route}/stage/cost")
        assert_true(stage.get("accepted_outer_iterations", 0) > 0, f"structural/{route}/stage/progress")
        assert_true(finite(stage.get("initial_cost")) and finite(stage.get("final_cost")), f"structural/{route}/stage/finite cost")
        assert_true(stage["final_cost"] < stage["initial_cost"], f"structural/{route}/stage/decrease")
        assert_true(main.get("linear_solver") == "MULTIFRONTAL_QR", f"structural/{route}/main/QR")
        assert_true(main.get("accepted_outer_iterations", 0) > 0, f"structural/{route}/main/progress")
        assert_true(main.get("strict_cost_decrease"), f"structural/{route}/main/cost")
        assert_true(finite(main.get("initial_cost")) and finite(main.get("final_cost")), f"structural/{route}/main/finite cost")
        assert_true(main["final_cost"] < main["initial_cost"], f"structural/{route}/main/decrease")
        for key in ("pseudorange_admitted_or_inserted", "doppler_admitted_or_inserted", "ordinary_tdcp_admitted_or_inserted", "pose3_x_bridge"):
            assert_true(family.get(key, 0) > 0, f"structural/{route}/family/{key}")
        assert_true(family.get("configuration_valid"), f"structural/{route}/family/configuration")
        assert_true(family.get("single_sagnac_representation"), f"structural/{route}/family/sagnac")
        assert_true(family.get("key_order_exact"), f"structural/{route}/family/key_order")
        assert_true(family.get("finite_jacobians_and_source_geometry"), f"structural/{route}/family/finite")
        assert_equal(c7d.get("c_dimension"), 7, f"structural/{route}/C dimension")
        assert_equal(c7d.get("c_units"), "metres", f"structural/{route}/C units")
        assert_equal(c7d.get("d_units"), "metres_per_second", f"structural/{route}/D units")
        assert_equal(c7d.get("c0d_sigma_m"), 0.1, f"structural/{route}/CCDD sigma")
        assert_equal(c7d.get("global_isb_state_count"), 0, f"structural/{route}/global ISB")
        assert_true(c7d.get("exact_epoch_alignment"), f"structural/{route}/C/D alignment")
        assert_true(base.get("applied") and base.get("built"), f"structural/{route}/base active")
        assert_equal(base.get("correction_application_pass_count"), 1, f"structural/{route}/base pass count")
        assert_true(base.get("correction_applied_exactly_once"), f"structural/{route}/base exactly once")
        assert_true(base.get("no_extrapolation_or_endpoint_hold"), f"structural/{route}/base no extrapolation")
        assert_true(tdcp.get("enabled"), f"structural/{route}/TDCP active")
        assert_equal(tdcp.get("fixed_sigma_m"), 0.03, f"structural/{route}/TDCP sigma")
        assert_equal(tdcp.get("official_huber_k"), 0.5, f"structural/{route}/TDCP Huber")
        assert_equal(tdcp.get("nonfinite_residuals"), 0, f"structural/{route}/TDCP finite")
        assert_true(output.get("finite_coordinates"), f"structural/{route}/output finite")
        assert_true(output.get("expected_epoch_coverage"), f"structural/{route}/output coverage")
        assert_equal(output.get("pixel5_offset_applications"), 1, f"structural/{route}/offset")
    return result


def verify_freeze() -> dict[str, Any]:
    if not AUDIT.is_file() or not FREEZE.is_file():
        raise fail("Phase137 audit/freeze missing")
    assert_equal(sha256_static(AUDIT, "Phase137 audit"), AUDIT_SHA256, "audit/sha256")
    freeze = read_json(FREEZE, "Phase137 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase137-phase136-accuracy-gate-freeze.v1",
        "phase": 137,
        "execution_label": "Luna Max",
        "status": "frozen-before-phase137-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    authority = freeze.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("freeze/authority missing")
    static_pins = {
        "accuracy_audit": (AUDIT, AUDIT_SHA256, AUDIT_COMMIT),
        "phase136_structural_result": (STRUCTURAL_RESULT, STRUCTURAL_SHA256, STRUCTURAL_COMMIT),
        "phase134_metric_evaluator": (PHASE134_EVALUATOR, PHASE134_EVALUATOR_SHA256, PHASE134_EVALUATOR_COMMIT),
        "phase134_accuracy_freeze": (ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_accuracy_freeze_v1.json", PHASE134_FREEZE_SHA256, PHASE134_FREEZE_COMMIT),
        "phase118_metric_evaluator": (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256, PHASE118_EVALUATOR_COMMIT),
        "phase118_accuracy_freeze": (ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_gate_freeze_v1.json", PHASE118_FREEZE_SHA256, PHASE118_FREEZE_COMMIT),
        "phase118_sealed_result": (ROOT / "docs/use_cases/records/smartphone_r5_phase118_tdcp_robust_k_accuracy_result_v1.json", PHASE118_RESULT_SHA256, PHASE118_FREEZE_COMMIT),
        "phase112_sealed_baseline": (ROOT / "docs/use_cases/records/smartphone_r5_phase112_main_output_offset_accuracy_result_v1.json", PHASE112_RESULT_SHA256, PHASE112_COMMIT),
        "phase82_scorer": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, PHASE82_EVALUATOR_COMMIT),
        "phase76_truth_parser": (PHASE76_PARSER, PHASE76_PARSER_SHA256, PHASE76_PARSER_COMMIT),
    }
    for key, (path, digest, commit) in static_pins.items():
        _pin(authority, key, path, digest)
        assert_equal(authority[key].get("commit"), commit, f"freeze/authority/{key}/commit")
        assert_equal(sha256_static(path, f"freeze/{key}"), digest, f"freeze/authority/{key}/sha256")
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
    assert_equal(candidate.get("route_order"), list(ROUTES), "freeze/candidate/routes")
    offset = candidate.get("phase136_pixel5_offset_boundary")
    if not isinstance(offset, Mapping):
        raise fail("freeze/candidate/phase136_pixel5_offset_boundary missing")
    for key, expected in {
        "already_applied_in_structural_run": True,
        "evaluator_must_not_apply_again": True,
        "application_count": 1,
        "double_application": "fail-closed",
    }.items():
        assert_equal(offset.get(key), expected, f"freeze/offset/{key}")
    metadata = candidate.get("routes_metadata")
    if not isinstance(metadata, Mapping):
        raise fail("freeze/candidate/routes_metadata missing")
    expected_opaque = {
        ROUTES[0]: ("dad4b5ee82942e11b051155f76fd28fe59c21e2928caba0d48a8955a6024005a", 172694, 2159),
        ROUTES[1]: ("df022cad3fdd68a3de2a6b2eceac0c0520a5a22fda61c76edf997efde964b519", 117254, 1466),
    }
    for route in ROUTES:
        item = metadata.get(route)
        if not isinstance(item, Mapping):
            raise fail(f"freeze candidate metadata missing: {route}")
        path = _deferred_path(item.get("path"), "opaque_solution_output.csv", f"candidate {route}", "/phase135-official-affine-structural-v1/")
        assert_equal(path, f"output/smartphone-r5/phase135-official-affine-structural-v1/{route.replace('/', '__')}/opaque_solution_output.csv", f"freeze/candidate/{route}/path")
        digest, bytes_count, newline_count = expected_opaque[route]
        assert_equal(item.get("sha256"), digest, f"freeze/candidate/{route}/sha256")
        assert_equal(item.get("bytes"), bytes_count, f"freeze/candidate/{route}/bytes")
        assert_equal(item.get("newline_count"), newline_count, f"freeze/candidate/{route}/newline_count")
        assert_equal(item.get("expected_prediction_rows"), DOMAIN_ROWS[route], f"freeze/candidate/{route}/prediction rows")
        assert_equal(item.get("expected_problem_epochs"), PROBLEM_EPOCHS[route], f"freeze/candidate/{route}/problem epochs")
        assert_equal(item.get("solution_rows_sealed"), PROBLEM_EPOCHS[route], f"freeze/candidate/{route}/sealed rows")
        assert_equal(item.get("opaque_metadata_seal"), True, f"freeze/candidate/{route}/opaque seal")
        assert_equal(item.get("bytes_and_rows_probed_before_authorization"), False, f"freeze/candidate/{route}/pre-auth probe")
        structural_item = structural["route_results"][STRUCTURAL_LABELS[route]]["opaque_solution"]
        assert_equal(structural_item.get("sha256"), digest, f"freeze/candidate/{route}/structural hash")
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
    assert_true(baselines.get("informational_only"), "freeze/baselines/informational")
    gates = freeze.get("promotion_gates")
    if not isinstance(gates, Mapping):
        raise fail("freeze/promotion_gates missing")
    for key, expected in {
        "all_gates_anded": True, "exact_two_routes_one_evaluation_each": True,
        "candidate_output_schema_and_exact_alignment": True,
        "candidate_prediction_domain_coverage_exact": 1.0,
        "candidate_all_finite_and_earth_valid": True,
        "candidate_over_70_mps_count_zero": True,
        "pixel5_offset_already_exactly_once_no_reapplication": True,
        "strict_less_than_is_explicit_and_gate": True,
        "strict_macro_comparator": "candidate_macro_score_m < 0.782",
        "strict_macro_threshold_m": 0.782, "equality_at_threshold_passes": False,
        "truth_read_only_by_one_evaluator_process": True,
        "solution_rows_absent_from_result": True,
        "no_solver_raw_base_mat_pdc_precomputed_or_kaggle": True,
    }.items():
        assert_equal(gates.get(key), expected, f"freeze/promotion_gates/{key}")
    boundary = freeze.get("truth_evaluator_boundary")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/truth_evaluator_boundary missing")
    for key, expected in {
        "authorization_required_before_candidate_or_truth_path_materialization": True,
        "candidate_metadata_hash_seal_precedes_truth": True,
        "candidate_parse_precedes_truth": True, "evaluator_process_count": 1,
        "candidate_solution_reads_for_hash_and_parse": 2,
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
        "candidate_solution_reads_for_hash_and_parse": 2,
        "candidate_coordinate_interpretations": 2, "truth_reads": 2,
        "truth_reads_per_route": 1, "accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0,
        "raw_base_rinex_reads": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(planned.get(key), expected, f"freeze/planned/{key}")
    return freeze


def verify_manifest() -> dict[str, Any]:
    freeze = verify_freeze()
    manifest = read_json(MANIFEST, "Phase137 manifest")
    for key, expected in {
        "schema_version": MANIFEST_SCHEMA, "phase": 137,
        "execution_label": "Luna Max",
        "status": "sealed-before-phase137-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/routes")
    assert_equal(manifest.get("candidate_reference"), {"source": "freeze.candidate", "candidate_count": 1, "route_order": list(ROUTES)}, "manifest/candidate_reference")
    assert_equal(manifest.get("truth_reference"), {"source": "freeze.truth_cohort", "route_order": list(ROUTES), "truth_payload_reads_before_authorization": 0}, "manifest/truth_reference")
    _verify_metric_contract(manifest.get("metric_contract"), "manifest/metric_contract")
    assert_equal(manifest.get("comparison_baselines"), freeze["comparison_baselines"], "manifest/comparison_baselines")
    assert_equal(manifest.get("freeze"), {"path": relative(FREEZE), "commit": FREEZE_COMMIT, "sha256": sha256_static(FREEZE, "Phase137 freeze")}, "manifest/freeze")
    assert_equal(manifest.get("structural_result"), {"path": relative(STRUCTURAL_RESULT), "commit": STRUCTURAL_COMMIT, "sha256": STRUCTURAL_SHA256}, "manifest/structural_result")
    authority = manifest.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("manifest/authority missing")
    expected_authority = {
        "accuracy_audit": (AUDIT, AUDIT_SHA256, AUDIT_COMMIT),
        "accuracy_freeze": (FREEZE, sha256_static(FREEZE, "Phase137 freeze"), FREEZE_COMMIT),
        "phase136_structural_result": (STRUCTURAL_RESULT, STRUCTURAL_SHA256, STRUCTURAL_COMMIT),
        "phase134_metric_evaluator": (PHASE134_EVALUATOR, PHASE134_EVALUATOR_SHA256, PHASE134_EVALUATOR_COMMIT),
        "phase118_metric_evaluator": (PHASE118_EVALUATOR, PHASE118_EVALUATOR_SHA256, PHASE118_EVALUATOR_COMMIT),
        "phase82_scorer": (PHASE82_EVALUATOR, PHASE82_EVALUATOR_SHA256, PHASE82_EVALUATOR_COMMIT),
        "phase76_truth_parser": (PHASE76_PARSER, PHASE76_PARSER_SHA256, PHASE76_PARSER_COMMIT),
    }
    for key, (path, digest, commit) in expected_authority.items():
        _pin(authority, key, path, digest)
        assert_equal(authority[key].get("commit"), commit, f"manifest/authority/{key}/commit")
        assert_equal(sha256_static(path, f"manifest/{key}"), digest, f"manifest/authority/{key}/sha256")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, Mapping):
        raise fail("manifest/implementation missing")
    for key, path in (("evaluator", EVALUATOR), ("focused_tests", ROOT / "tests/test_smartphone_phase137_phase136_accuracy.py")):
        item = implementation.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest/implementation/{key} missing")
        assert_equal(item.get("path"), relative(path), f"manifest/implementation/{key}/path")
        digest = item.get("sha256")
        assert_true(isinstance(digest, str) and len(digest) == 64, f"manifest/implementation/{key}/sha256")
        assert_equal(sha256_static(path, f"manifest/implementation/{key}"), digest, f"manifest/implementation/{key}/sha256")
    _verify_zero_accounting(manifest.get("read_accounting_before_authorization"), "manifest/read_accounting_before_authorization")
    requirements = manifest.get("authorization_requirements")
    if not isinstance(requirements, Mapping):
        raise fail("manifest/authorization_requirements missing")
    for key, expected in {
        "authorization_file": relative(AUTHORIZATION), "schema_version": AUTHORIZATION_SCHEMA,
        "status": "authorized-for-phase137-truth-only-accuracy-evaluation",
        "independent_commit_required": True,
        "candidate_and_truth_paths_materialized_only_after_auth": True,
        "route_order": list(ROUTES), "exact_candidate_reads": 2,
        "exact_truth_reads": 2, "exact_accuracy_calculations": 2,
        "native_solver_invocations": 0, "raw_reads": 0, "base_reads": 0,
        "reruns": 0, "fallbacks": 0, "solution_publication": False,
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0,
    }.items():
        assert_equal(requirements.get(key), expected, f"manifest/authorization/{key}")
    return manifest


def verify_pre_truth() -> dict[str, Any]:
    manifest = verify_manifest()
    return {
        "status": "pre-truth-verified", "phase": 137, "execution_label": "Luna Max",
        "route_order": list(ROUTES), "candidate_count": manifest["candidate_reference"]["candidate_count"],
        "candidate_paths_materialized": 0, "truth_paths_materialized": 0,
        "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0,
        "truth_reads": 0, "accuracy_calculations": 0, "native_solver_invocations": 0,
        "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0,
        "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0,
        "reruns": 0, "fallbacks": 0, "truth_only_authorized": False,
        "raw_execution_or_solver_authorized": False, "solution_output_published": False,
    }


def _verify_auth_pin(authority: Mapping[str, Any], key: str, path: Path) -> None:
    item = authority.get(key)
    if not isinstance(item, Mapping):
        raise fail(f"authorization/{key} pin missing")
    digest = item.get("sha256")
    assert_true(isinstance(digest, str) and len(digest) == 64, f"authorization/{key}/sha256")
    _pin(authority, key, path, digest)
    assert_equal(sha256_static(path, f"authorization/{key}"), digest, f"authorization/{key}/sha256")


def verify_authorization() -> dict[str, Any]:
    """Verify future independent authorization without payload probing."""
    manifest = verify_manifest()
    auth = read_json(AUTHORIZATION, "Phase137 truth-only authorization")
    for key, expected in {
        "schema_version": AUTHORIZATION_SCHEMA, "phase": 137,
        "execution_label": "Luna Max",
        "status": "authorized-for-phase137-truth-only-accuracy-evaluation",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    authority = auth.get("authority")
    if not isinstance(authority, Mapping):
        raise fail("authorization/authority missing")
    pinned = {
        "accuracy_audit": AUDIT, "accuracy_freeze": FREEZE,
        "accuracy_manifest": MANIFEST, "phase136_structural_result": STRUCTURAL_RESULT,
        "accuracy_evaluator": EVALUATOR,
        "focused_tests": ROOT / "tests/test_smartphone_phase137_phase136_accuracy.py",
        "phase134_metric_evaluator": PHASE134_EVALUATOR,
        "phase118_metric_evaluator": PHASE118_EVALUATOR,
        "phase82_scorer": PHASE82_EVALUATOR, "phase76_truth_parser": PHASE76_PARSER,
    }
    for key, path in pinned.items():
        _verify_auth_pin(authority, key, path)
    assert_equal(auth.get("routes"), list(ROUTES), "authorization/routes")
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
        "mat_precomputed_phone_coordinate_pdc_reads": 0,
        "kaggle_or_token_access": 0, "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(policy.get(key), expected, f"authorization/policy/{key}")
    assert_equal(auth.get("manifest_sha256"), sha256_static(MANIFEST, "Phase137 manifest"), "authorization/manifest_sha256")
    return auth


def materialize_candidate_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("candidate path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = _deferred_path(freeze["candidate"]["routes_metadata"][route]["path"], "opaque_solution_output.csv", f"candidate {route}", "/phase135-official-affine-structural-v1/")
    return ROOT / text


def materialize_truth_path(freeze: Mapping[str, Any], route: str, *, authorized: bool) -> Path:
    if not authorized:
        raise fail("truth path materialization requires independent truth authorization")
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    text = _deferred_path(freeze["truth_cohort"]["routes"][route]["path"], "ground_truth.csv", f"truth {route}", "/truth/")
    return ROOT / text


def load_phase118() -> Any:
    assert_equal(sha256_static(PHASE118_EVALUATOR, "Phase118 evaluator"), PHASE118_EVALUATOR_SHA256, "Phase118 evaluator/sha256")
    return _load_module(PHASE118_EVALUATOR, "phase118_accuracy_reference_phase137")


def read_candidate_once(path: Path, seal: Mapping[str, Any], route: str, p82: Any) -> tuple[list[tuple[int, float, float]], dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != seal.get("sha256") or len(payload) != seal.get("bytes"):
        raise fail(f"Phase137 candidate opaque seal mismatch: {route}")
    if payload.count(b"\n") != seal.get("newline_count"):
        raise fail(f"Phase137 candidate newline seal mismatch: {route}")
    ordered, mapping = p82.P76.P74._parse_submission(payload, route)
    if len(ordered) != DOMAIN_ROWS[route] or len(mapping) != len(ordered) or [item[0] for item in ordered] != sorted(item[0] for item in ordered):
        raise fail(f"Phase137 candidate exact alignment failed: {route}")
    if any(not all(math.isfinite(float(value)) for value in item[1:]) or not -90.0 <= item[1] <= 90.0 or not -180.0 <= item[2] <= 180.0 for item in ordered):
        raise fail(f"Phase137 candidate finite/Earth-valid preflight failed: {route}")
    return ordered, mapping, {"bytes": len(payload), "sha256": digest, "rows": len(ordered), "read_count": 1, "coordinate_rows_omitted": True}


def read_truth_once(path: Path, pin: Mapping[str, Any], route: str, p82: Any) -> tuple[dict[int, tuple[float, float]], dict[str, Any]]:
    payload = path.read_bytes()
    digest = hashlib.sha256(payload).hexdigest()
    if digest != pin.get("sha256") or len(payload) != pin.get("bytes"):
        raise fail(f"Phase137 truth seal mismatch: {route}")
    truth = p82.P76._parse_truth_dictreader(payload, route)
    if len(truth) != pin.get("rows"):
        raise fail(f"Phase137 truth row count mismatch: {route}")
    return truth, {"bytes": len(payload), "sha256": digest, "rows": len(truth), "read_count": 1, "coordinate_rows_omitted": True}


def strict_macro_gate(value: Any) -> bool:
    return finite(value) and float(value) < STRICT_PROMOTION_THRESHOLD_M


def evaluate(result_path: Path = RESULT_JSON) -> dict[str, Any]:
    """Future isolated lane; authorization precedes exactly four payload reads."""
    manifest = verify_manifest()
    verify_authorization()
    freeze = verify_freeze()
    structural = verify_structural_result()
    p118 = load_phase118()
    p82 = p118.load_phase82()
    baseline = manifest["comparison_baselines"]["phase112_route_scores_m"]
    reports: dict[str, Any] = {}
    accounting = {"candidate_paths_materialized": 0, "truth_paths_materialized": 0, "candidate_solution_reads": 0, "candidate_coordinate_interpretations": 0, "truth_reads": 0, "accuracy_calculations": 0}
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
            "candidate_finite_and_earth_valid": score.get("finite") is True,
            "candidate_over_70_mps_count_zero": score.get("over_70_mps_count") == 0,
            "candidate_route_score_finite": finite(score.get("score_m")),
            "candidate_route_score_at_most_3m": finite(score.get("score_m")) and score.get("score_m") <= 3.0,
            "candidate_no_route_regression_vs_phase112": finite(score.get("score_m")) and score.get("score_m") <= float(baseline[route]),
        }
        reports[route] = {"dataset_id": route, "candidate_solution": candidate_meta, "truth": truth_meta, "truth_read": True, "accuracy_scored": True, "candidate": score, "baseline_phase112": {"score_m": baseline[route]}, "gates": {"passed": all(checks.values()), "checks": checks, "failures": [key for key, value in checks.items() if value is not True]}}
    candidate_macro = sum(float(reports[route]["candidate"]["score_m"]) for route in ROUTES) / 2.0
    gates = {
        "exact_two_routes_one_evaluation_each": accounting == {"candidate_paths_materialized": 2, "truth_paths_materialized": 2, "candidate_solution_reads": 2, "candidate_coordinate_interpretations": 2, "truth_reads": 2, "accuracy_calculations": 2},
        "candidate_output_schema_and_exact_alignment": all(reports[route]["gates"]["checks"]["candidate_schema_and_exact_alignment"] for route in ROUTES),
        "candidate_prediction_domain_coverage_exact": all(reports[route]["candidate"].get("prediction_domain_coverage") == 1.0 for route in ROUTES),
        "candidate_all_finite_and_earth_valid": all(reports[route]["candidate"].get("finite") is True for route in ROUTES),
        "candidate_over_70_mps_count_zero": all(reports[route]["candidate"].get("over_70_mps_count") == 0 for route in ROUTES),
        "candidate_route_scores_finite": all(finite(reports[route]["candidate"].get("score_m")) for route in ROUTES),
        "candidate_each_route_score_at_most_3m": all(reports[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES),
        "candidate_no_route_regression_vs_phase112": all(reports[route]["candidate"]["score_m"] <= float(baseline[route]) for route in ROUTES),
        "candidate_macro_score_strict_less_than_0_782m": strict_macro_gate(candidate_macro),
        "truth_read_only_by_one_evaluator_process": accounting["truth_reads"] == 2,
        "no_solution_rows_in_result": True,
        "no_solver_raw_base_mat_precomputed_pdc_kaggle": True,
    }
    failed = [key for key, value in gates.items() if value is not True]
    result = {
        "schema_version": RESULT_SCHEMA, "phase": 137, "execution_label": "Luna Max",
        "status": "go-phase137-truth-only-accuracy" if not failed else "no-go-phase137-truth-only-accuracy",
        "decision": "truth-only accuracy evaluated; release remains separately unauthorized" if not failed else "truth-only accuracy gate failed closed; preserve artifacts and do not rerun",
        "candidate": CANDIDATE_ID, "accuracy_scored": True, "routes": reports,
        "aggregate": {"candidate_macro_score_m": candidate_macro, "phase112_macro_m": manifest["comparison_baselines"]["phase112_macro_m"], "phase118_champion_macro_m": manifest["comparison_baselines"]["phase118_champion_macro_m"], "route_count": 2, "macro_route_order": list(ROUTES), "macro_weighting": "unweighted arithmetic mean over exactly MTV-A then LAX-T"},
        "metric_contract": manifest["metric_contract"], "promotion_gates": gates,
        "strict_0_782_gate": {"comparator": "candidate_macro_score_m < 0.782", "threshold_m": STRICT_PROMOTION_THRESHOLD_M, "candidate_macro_score_m": candidate_macro, "passed": gates["candidate_macro_score_strict_less_than_0_782m"]},
        "failed_gates": failed, "read_accounting": {"native_solver_invocations": 0, "raw_gnss_imu_navigation_reads": 0, "raw_base_rinex_reads": 0, "candidate_solution_reads_for_hash_and_parse": accounting["candidate_solution_reads"], "candidate_coordinate_interpretations": accounting["candidate_coordinate_interpretations"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "accuracy_calculations": accounting["accuracy_calculations"], "candidate_paths_materialized_after_authorization": accounting["candidate_paths_materialized"], "truth_paths_materialized_after_authorization": accounting["truth_paths_materialized"], "reruns": 0, "fallbacks": 0, "mat_precomputed_phone_coordinate_pdc_reads": 0, "kaggle_or_token_access": 0, "truth_reads_by_process": "one Phase137 truth-only evaluator process"},
        "forbidden_lanes": {"native_solver_rerun": False, "raw_or_base_rerun": False, "mat": False, "precomputed_phone_coordinates": False, "pdc": False, "kaggle_or_token": False, "solution_rows_in_result": False},
        "solution_output_published": False, "release_or_submission_authorized": False,
    }
    atomic_json(result_path, result)
    return result


def main(argv: list[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-truth", action="store_true")
    parser.add_argument("--evaluate", action="store_true")
    parser.add_argument("--result-json", type=Path, default=RESULT_JSON)
    args = parser.parse_args(argv)
    modes = [args.verify_freeze, args.verify_manifest, args.verify_pre_truth, args.evaluate]
    if sum(bool(mode) for mode in modes) != 1:
        parser.error("choose exactly one verification/evaluation mode")
    try:
        if args.verify_freeze:
            verify_freeze()
        elif args.verify_manifest:
            verify_manifest()
        elif args.verify_pre_truth:
            print(json.dumps(verify_pre_truth(), indent=2, sort_keys=True))
        else:
            result = evaluate(args.result_json)
            print(json.dumps(result, indent=2, sort_keys=True))
            return 0 if result.get("status", "").startswith("go-") else 1
        return 0
    except Exception as exc:
        print(f"phase137 truth-only evaluator: fail-closed: {exc}", file=__import__("sys").stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
