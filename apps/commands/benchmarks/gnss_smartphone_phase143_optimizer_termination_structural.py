#!/usr/bin/env python3
"""Launch-free Phase143 optimizer-termination structural contract.

This validator checks only pinned source, the Phase143 structural freeze and
manifest, and synthetic/in-memory structural summaries.  It never materializes
or reads raw GNSS/IMU/navigation/base payloads, solution coordinates, truth,
MAT/PDC/precomputed artifacts, and never starts the native solver.  A new
independent authorization is required before any future raw run.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_structural_audit_v1.md"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_structural_freeze_v1.json"
)
DESIGN_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_parity_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_structural_manifest_v1.json"
)
PRE_RAW = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_structural_pre_raw_v1.json"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase143_optimizer_termination_structural.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase143_optimizer_termination_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
PHASE138_RUNNER = ROOT / (
    "apps/commands/benchmarks/gnss_smartphone_phase138_affine_tdcp_structural.py"
)


AUDIT_COMMIT = "11911df8f0c791af136c7892c46d547f28dbeee8"
AUDIT_SHA256 = "05f83e2a22e697616e5d1c53b6e19e518658b9d4fe85abb10ac2c31ab43e3f8a"
DESIGN_FREEZE_COMMIT = "03073a14590f0c6316e7a0294f6b575b796b7c11"
DESIGN_FREEZE_SHA256 = "9d579969bdd2002f251db16c0624d03da6fd2d5d33e751f0d1e0ac5d40b3a33f"
STRUCTURAL_FREEZE_COMMIT = "95a24271ae3aa1c8dc64785f19e3aa8a475c8beb"
STRUCTURAL_FREEZE_SHA256 = "dbaa37376e2c1e743b1f11500a4a0cf6fa04fc4307a68df259111435c8e81ce3"
IMPLEMENTATION_COMMIT = "f07a80bb7e26fb0502454b3cb8dc291c53545714"
TARGET_BINARY_SHA256 = "15ab7b401f16476928643bf41485e168a4a8be28bdb6fd1b2f8f7e26e166ef20"

SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp":
        "085cedec22d58158a760a5d06acf63271f82c9d98025a81ed3b93618966bef2a",
    "include/libgnss++/algorithms/fgo.hpp":
        "b24ea504dea3a68583659939f74fa4766e10dc5c96f727c1bb3a57d593263d00",
    "include/libgnss++/algorithms/fgo_config.hpp":
        "f982cb0468bab27222634a76953cda00a7ece6a360bc38ed4e25308c45e1ca12",
    "src/algorithms/fgo_gtsam_backend.cpp":
        "1d1e7819fe20c163d336ac4acc8b0fb6c7fdb3385096962330e894be56b22399",
    "src/algorithms/fgo_gtsam_internal.hpp":
        "be61a643c181206c3260549c05407e81b6a7932c81357a82e4de379b5bf560d5",
}

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
ROUTE_LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
SCHEMA = "smartphone-r5-phase143-optimizer-termination-structural-manifest.v1"
NATIVE_TERMINATION_SCHEMA = "smartphone-r5-native-fgo-phase143-termination.v1"

PHASE135 = "--native-phase135-official-affine-measurement-family"
PHASE138 = "--native-phase138-affine-tdcp-anchor-range-constant"
PHASE118 = "--native-phase118-official-tdcp-huber-k"
PHASE143 = "--native-phase143-official-main-lm-termination-budget"
PHASE117 = "--native-phase117-tdcp-snr-type-sigma"
PHASE120 = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
PHASE126 = "--native-phase126-raw-base-source-complete"
PHASE127 = "--native-phase127-glonass-channel-provenance"
PHASE128 = "--native-phase128-glonass-provenance-parser-admission"
PHASE129 = "--native-phase129-glonass-local-miss-mask"
PHASE130 = "--native-phase130-shared-ledger-key-local-support"
PHASE131 = "--native-phase131-canonical-correction-band-key"
ADDITIONAL = "--native-base-pseudorange-preserve-additional-frequency-bands"

RAW_PLACEHOLDERS = {
    "--android-gnss": "__PHASE143_RAW_DEVICE_GNSS__",
    "--android-imu": "__PHASE143_RAW_DEVICE_IMU__",
    "--nav": "__PHASE143_RAW_BROADCAST_NAV__",
    "--native-base-rinex": "__PHASE143_RAW_BASE_RINEX__",
    "--native-base-rinex-sha256": "__PHASE143_RAW_BASE_SHA256__",
}
REQUIRED_RECIPE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    # Existing native algorithm switch; this is not a PDC input or path.
    "--native-pdc-imu-tdcp-no-bridge",
    "--native-source-direct-observable-quality",
    "--native-source-clock-c0d-factor",
    "--native-source-clock-c0d-meter-state-parity",
    "--native-source-clock-c0d-active-solve-diagnostic",
    "--native-source-clock-c0d-gnss-first-meter-state-handoff",
    "--native-source-clock-c0d-epoch-vector-parity",
    "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-upstream-position-offset",
)
ON_SELECTORS = (PHASE135, PHASE138, PHASE118, PHASE143)
OFF_SELECTORS = (
    PHASE117, PHASE120, PHASE126, PHASE127, PHASE128, PHASE129, PHASE130,
    PHASE131, ADDITIONAL,
)
ALLOWED_TERMINATION_BRANCHES = {
    "maximum_outer_iterations",
    "outer_convergence_tolerance",
    "small_cost_change",
    "maximum_lambda",
    "no_inner_iteration",
    "exception",
    "no_progress_unclassified",
}


class Phase143ContractError(ValueError):
    """A Phase143 contract violation which must fail closed."""


def fail(message: str) -> Phase143ContractError:
    return Phase143ContractError(message)


def _reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key: {key}")
        result[key] = value
    return result


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=_reject_duplicate_pairs
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, Phase143ContractError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} must be an object")
    return value


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def static_sha256(path: Path, label: str) -> str:
    """Hash only tracked static artifacts; payload hashes are forbidden."""
    lowered = path.name.lower()
    if lowered.endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"payload hash forbidden for {label}")
    if any(term in lowered for term in ("truth", "ground_truth", "precomputed")):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def _true(mapping: Mapping[str, Any], key: str, label: str) -> None:
    value = _required(mapping, key, label)
    if value is not True:
        raise fail(f"{label}/{key}: expected true")


def _bool(mapping: Mapping[str, Any], key: str, label: str) -> bool:
    value = _required(mapping, key, label)
    if not isinstance(value, bool):
        raise fail(f"{label}/{key}: expected boolean")
    return value


def _count(mapping: Mapping[str, Any], key: str, label: str) -> int:
    value = _required(mapping, key, label)
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise fail(f"{label}/{key}: expected nonnegative integer")
    return value


def _finite(mapping: Mapping[str, Any], key: str, label: str) -> float:
    value = _required(mapping, key, label)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise fail(f"{label}/{key}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        raise fail(f"{label}/{key}: expected finite number")
    return result


def _zero_reads(mapping: Mapping[str, Any], label: str) -> None:
    if not isinstance(mapping, Mapping):
        raise fail(f"{label}: expected object")
    for key, value in mapping.items():
        if isinstance(value, bool):
            _equal(value, False, f"{label}/{key}")
        elif isinstance(value, int):
            _equal(value, 0, f"{label}/{key}")
        else:
            raise fail(f"{label}/{key}: expected zero")


def _load_phase138_runner() -> Any:
    spec = importlib.util.spec_from_file_location("phase138_contract_dependency", PHASE138_RUNNER)
    if spec is None or spec.loader is None:
        raise fail("unable to load retained Phase138 structural validator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def command_template(route: str) -> list[str]:
    if route not in ROUTES:
        raise fail(f"unknown route {route!r}")
    route_dir = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase143-optimizer-termination-structural-v1"
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id", route,
        "--android-gnss", RAW_PLACEHOLDERS["--android-gnss"],
        "--android-imu", RAW_PLACEHOLDERS["--android-imu"],
        "--nav", RAW_PLACEHOLDERS["--nav"],
        *REQUIRED_RECIPE_FLAGS,
        *ON_SELECTORS,
        "--native-base-rinex", RAW_PLACEHOLDERS["--native-base-rinex"],
        "--native-base-rinex-sha256", RAW_PLACEHOLDERS["--native-base-rinex-sha256"],
        "--out",
        f"{output_root}/{route_dir}/opaque_solution_output.csv",
        "--summary-json",
        f"{output_root}/{route_dir}/native_summary.json",
    ]


def validate_command(route: str, command: Any) -> None:
    if not isinstance(command, list) or any(not isinstance(item, str) for item in command):
        raise fail(f"command/{route}: expected argv list")
    _equal(command, command_template(route), f"command/{route}")
    for selector in REQUIRED_RECIPE_FLAGS + ON_SELECTORS:
        _equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in OFF_SELECTORS:
        _equal(command.count(selector), 0, f"command/{route}/{selector}")
    for flag, placeholder in RAW_PLACEHOLDERS.items():
        _equal(command[command.index(flag) + 1], placeholder,
               f"command/{route}/{flag}/placeholder")
    summary_path = command[command.index("--summary-json") + 1]
    if not summary_path.endswith("/native_summary.json"):
        raise fail(f"command/{route}: summary path is not native_summary.json")
    if "structural_summary" in summary_path:
        raise fail(f"command/{route}: native/structural summary paths are conflated")


def validate_freeze(freeze: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": "smartphone-r5-phase143-optimizer-termination-structural-freeze.v1",
        "phase": 143,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-structural-contract",
    }.items():
        _equal(freeze.get(key), expected, f"freeze/{key}")
    audit = _required(freeze, "audit", "freeze")
    if not isinstance(audit, Mapping):
        raise fail("freeze/audit: expected object")
    for key, expected in {
        "path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256,
    }.items():
        _equal(audit.get(key), expected, f"freeze/audit/{key}")
    design = _required(freeze, "design_freeze", "freeze")
    if not isinstance(design, Mapping):
        raise fail("freeze/design_freeze: expected object")
    _equal(design.get("path"), relative(DESIGN_FREEZE), "freeze/design/path")
    _equal(design.get("commit"), DESIGN_FREEZE_COMMIT, "freeze/design/commit")
    _equal(design.get("sha256"), DESIGN_FREEZE_SHA256, "freeze/design/sha256")
    implementation = _required(freeze, "implementation", "freeze")
    if not isinstance(implementation, Mapping):
        raise fail("freeze/implementation: expected object")
    _equal(implementation.get("commit"), IMPLEMENTATION_COMMIT,
           "freeze/implementation/commit")
    _equal(implementation.get("target_binary_sha256"), TARGET_BINARY_SHA256,
           "freeze/implementation/binary")
    candidate = _required(freeze, "candidate", "freeze")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate: expected object")
    for key, expected in {
        "count": 1,
        "id": "phase143-official-main-lm-termination-budget-v1",
        "selector": PHASE143,
        "default_off": True,
        "partial_selector_set_allowed": False,
        "raw_execution_authorized": False,
        "solver_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "solution_publication_authorized": False,
        "wrapper_inference": False,
        "solution_or_accuracy_fields": False,
    }.items():
        _equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    recipe = _required(freeze, "recipe", "freeze")
    if not isinstance(recipe, Mapping):
        raise fail("freeze/recipe: expected object")
    _equal(recipe.get("on_selectors"), list(ON_SELECTORS), "freeze/recipe/on")
    _equal(recipe.get("off_selectors"), list(OFF_SELECTORS), "freeze/recipe/off")
    _equal(recipe.get("phase107_raw_base_compensation"), True,
           "freeze/recipe/phase107 raw base")
    _equal(recipe.get("phase107_raw_base_source_miss_mask"), True,
           "freeze/recipe/phase107 source miss mask")
    _equal(recipe.get("phase118_fixed_tdcp_sigma_m"), 0.03,
           "freeze/recipe/sigma")
    _equal(recipe.get("phase118_highway_huber_k"), 0.5,
           "freeze/recipe/Huber k")
    _equal(recipe.get("phase99_main_linear_solver"), "MULTIFRONTAL_QR",
           "freeze/recipe/solver")
    _equal(recipe.get("phase99_main_elimination"), "EliminateQR",
           "freeze/recipe/elimination")
    _equal(recipe.get("route_order"), list(ROUTES), "freeze/recipe/routes")
    _equal(recipe.get("runs_per_route"), 1, "freeze/recipe/runs")
    gates = _required(freeze, "structural_gates", "freeze")
    if not isinstance(gates, Mapping):
        raise fail("freeze/structural_gates: expected object")
    for key, value in gates.items():
        _equal(value, True, f"freeze/structural_gates/{key}")
    _zero_reads(_required(freeze, "read_accounting_before_authorization", "freeze"),
                "freeze/read_accounting_before_authorization")
    boundary = _required(freeze, "authorization_boundary", "freeze")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/authorization_boundary: expected object")
    for key in (
        "independent_raw_authorization_commit_required",
        "materialize_inputs_after_authorization_only",
        "route_order_fixed",
        "exactly_one_solver_per_route_max",
        "raw_result_commit_separate",
        "truth_accuracy_authorization_required_separately",
    ):
        _equal(boundary.get(key), True, f"freeze/authorization_boundary/{key}")
    _equal(boundary.get("solution_content_or_coordinate_read_authorized"), False,
           "freeze/authorization_boundary/solution")
    _equal(boundary.get("rerun_fallback_repair_sweep"), False,
           "freeze/authorization_boundary/retry")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 143,
        "status": "launch-free",
        "audit_commit": AUDIT_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "design_freeze_commit": DESIGN_FREEZE_COMMIT,
        "structural_freeze_commit": STRUCTURAL_FREEZE_COMMIT,
        "freeze_sha256": STRUCTURAL_FREEZE_SHA256,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "routes": list(ROUTES),
    }.items():
        _equal(manifest.get(key), expected, f"manifest/{key}")
    _equal(manifest.get("audit_path"), relative(AUDIT), "manifest/audit_path")
    _equal(manifest.get("freeze_path"), relative(FREEZE), "manifest/freeze_path")
    _equal(manifest.get("design_freeze_path"), relative(DESIGN_FREEZE),
           "manifest/design_freeze_path")
    _equal(manifest.get("pre_raw_path"), relative(PRE_RAW), "manifest/pre_raw_path")
    recipe = _required(manifest, "recipe", "manifest")
    if not isinstance(recipe, Mapping):
        raise fail("manifest/recipe: expected object")
    _equal(recipe.get("on_selectors"), list(ON_SELECTORS), "manifest/recipe/on")
    _equal(recipe.get("off_selectors"), list(OFF_SELECTORS), "manifest/recipe/off")
    for key, expected in {
        "phase107_raw_base_compensation": True,
        "phase107_raw_base_source_miss_mask": True,
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "phase99_main_linear_solver": "MULTIFRONTAL_QR",
        "phase99_main_elimination": "EliminateQR",
        "phase143_main_effective_max_iterations": 1000,
        "phase143_gnss_first_effective_max_iterations": 1000,
        "phase143_timeout_policy": "no artificial timeout shorter than native contract",
    }.items():
        _equal(recipe.get(key), expected, f"manifest/recipe/{key}")
    policy = _required(manifest, "policy", "manifest")
    if not isinstance(policy, Mapping):
        raise fail("manifest/policy: expected object")
    for key in (
        "raw_execution_authorized", "solver_execution_authorized", "truth_used",
        "mat_used", "pdc_used", "precomputed_coordinates_used", "accuracy_evaluation",
        "kaggle_access", "solution_publication", "rerun", "fallback", "repair", "sweep",
    ):
        _equal(policy.get(key), False, f"manifest/policy/{key}")
    _equal(policy.get("opaque_solution_only"), True, "manifest/policy/opaque_solution_only")
    _zero_reads(_required(manifest, "pre_raw_read_accounting", "manifest"),
                "manifest/pre_raw_read_accounting")
    commands = _required(manifest, "command_snapshots", "manifest")
    if not isinstance(commands, Mapping):
        raise fail("manifest/command_snapshots: expected object")
    for route in ROUTES:
        validate_command(route, commands.get(route))
    artifacts = _required(manifest, "artifacts", "manifest")
    if not isinstance(artifacts, Mapping):
        raise fail("manifest/artifacts: expected object")
    for key, path in (("audit", AUDIT), ("freeze", FREEZE),
                      ("design_freeze", DESIGN_FREEZE), ("runner", RUNNER),
                      ("focused_tests", FOCUSED_TESTS)):
        item = artifacts.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest/artifacts/{key}: expected object")
        _equal(item.get("path"), relative(path), f"manifest/artifacts/{key}/path")
        _equal(item.get("sha256"), static_sha256(path, f"manifest/artifacts/{key}"),
               f"manifest/artifacts/{key}/sha256")
    structural = _required(manifest, "structural_summary_schema", "manifest")
    if not isinstance(structural, Mapping):
        raise fail("manifest/structural_summary_schema: expected object")
    _equal(structural.get("termination_schema_version"), NATIVE_TERMINATION_SCHEMA,
           "manifest/structural_summary_schema/termination_schema_version")
    _equal(structural.get("solution_coordinate_fields_forbidden"), True,
           "manifest/structural_summary_schema/solution policy")


def validate_pre_raw(pre_raw: Mapping[str, Any], manifest: Mapping[str, Any]) -> None:
    _equal(pre_raw.get("schema_version"),
           "smartphone-r5-phase143-optimizer-termination-structural-pre-raw.v1",
           "pre_raw/schema_version")
    _equal(pre_raw.get("phase"), 143, "pre_raw/phase")
    _equal(pre_raw.get("status"), "sealed-launch-free-zero-read", "pre_raw/status")
    _equal(pre_raw.get("manifest_path"), relative(MANIFEST), "pre_raw/manifest_path")
    _equal(pre_raw.get("manifest_sha256"), static_sha256(MANIFEST, "manifest"),
           "pre_raw/manifest_sha256")
    _equal(pre_raw.get("freeze_commit"), STRUCTURAL_FREEZE_COMMIT,
           "pre_raw/freeze_commit")
    _equal(pre_raw.get("implementation_commit"), IMPLEMENTATION_COMMIT,
           "pre_raw/implementation_commit")
    _equal(pre_raw.get("target_binary_sha256"), TARGET_BINARY_SHA256,
           "pre_raw/binary_sha256")
    _equal(pre_raw.get("route_order"), list(ROUTES), "pre_raw/route_order")
    _equal(pre_raw.get("runs_per_route"), 1, "pre_raw/runs_per_route")
    _zero_reads(_required(pre_raw, "read_accounting", "pre_raw"),
                "pre_raw/read_accounting")
    policy = _required(pre_raw, "policy", "pre_raw")
    if not isinstance(policy, Mapping):
        raise fail("pre_raw/policy: expected object")
    for key in ("raw_execution_authorized", "solver_execution_authorized",
                "truth_used", "solution_coordinate_reads", "mat_pdc_precomputed_reads",
                "accuracy_evaluation", "kaggle_access", "rerun", "fallback", "repair"):
        _equal(policy.get(key), False, f"pre_raw/policy/{key}")
    _equal(pre_raw.get("native_binary_invoked"), False, "pre_raw/native_binary_invoked")
    _equal(pre_raw.get("raw_payload_materialized"), False, "pre_raw/raw_payload_materialized")
    _equal(pre_raw.get("solution_rows_opened"), False, "pre_raw/solution_rows_opened")
    _equal(pre_raw.get("truth_rows_opened"), False, "pre_raw/truth_rows_opened")
    _equal(pre_raw.get("authorization_required_before_raw"), True,
           "pre_raw/authorization_required_before_raw")
    _equal(pre_raw.get("new_independent_authorization_required"), True,
           "pre_raw/new_independent_authorization_required")
    # The argument is intentionally consumed to make the pin relationship
    # explicit without opening any route payload.
    _equal(manifest.get("pre_raw_path"), relative(PRE_RAW), "manifest/pre_raw_path")


def validate_termination_stage(stage: str, report: Mapping[str, Any]) -> None:
    label = f"termination/{stage}"
    if not isinstance(report, Mapping):
        raise fail(f"{label}: expected object")
    if any(key in report for key in ("solution_rows", "solution_values", "coordinate_rows")):
        raise fail(f"{label} contains solution content")
    required = (
        "selector_enabled", "stage", "configured_max_iterations",
        "effective_max_iterations", "attempted", "attempted_outer_iterations",
        "accepted_outer_iterations", "rejected_outer_iterations",
        "total_inner_lambda_attempts", "initial_cost", "final_cost", "costs_finite",
        "strict_cost_decrease", "termination_branch", "relative_error_tolerance",
        "absolute_error_tolerance", "error_tolerance", "initial_lambda", "final_lambda",
        "maximum_lambda", "lambda_factor", "lambda_lower_bound", "lambda_upper_bound",
        "min_model_fidelity", "diagonal_damping", "use_fixed_lambda_factor",
        "linear_solver", "elimination", "ordering_type", "explicit_ordering_present",
        "no_fallback", "termination_trace_complete", "configuration_valid",
    )
    for key in required:
        _required(report, key, label)
    _true(report, "selector_enabled", label)
    _equal(report.get("stage"), stage, f"{label}/stage")
    configured = _count(report, "configured_max_iterations", label)
    effective = _count(report, "effective_max_iterations", label)
    _equal(effective, 1000, f"{label}/effective_max_iterations")
    if stage == "main" and configured not in {12, 1000}:
        raise fail(f"{label}/configured_max_iterations: outside 12-to-1000 boundary")
    if stage == "gnss-first" and configured != 1000:
        raise fail(f"{label}/configured_max_iterations: expected 1000")
    _true(report, "attempted", label)
    attempted = _count(report, "attempted_outer_iterations", label)
    accepted = _count(report, "accepted_outer_iterations", label)
    rejected = _count(report, "rejected_outer_iterations", label)
    if accepted <= 0:
        raise fail(f"{label}/accepted_outer_iterations: zero")
    if accepted > attempted or accepted > effective:
        raise fail(f"{label}: accepted outer iterations exceed attempted/cap")
    _equal(rejected, attempted - accepted, f"{label}/outer iteration conservation")
    if _count(report, "total_inner_lambda_attempts", label) < accepted:
        raise fail(f"{label}/total_inner_lambda_attempts: below accepted count")
    initial = _finite(report, "initial_cost", label)
    final = _finite(report, "final_cost", label)
    for key in (
        "relative_error_tolerance", "absolute_error_tolerance", "error_tolerance",
        "initial_lambda", "final_lambda", "maximum_lambda", "lambda_factor",
        "lambda_lower_bound", "lambda_upper_bound", "min_model_fidelity",
    ):
        _finite(report, key, label)
    _true(report, "costs_finite", label)
    _true(report, "strict_cost_decrease", label)
    if not final < initial:
        raise fail(f"{label}: final cost did not strictly decrease")
    _equal(report.get("termination_branch") in ALLOWED_TERMINATION_BRANCHES, True,
           f"{label}/termination_branch")
    if report["termination_branch"] == "maximum_outer_iterations":
        _equal(accepted, effective, f"{label}/maximum_outer_iterations cap")
    if report["termination_branch"] == "no_inner_iteration":
        # A raw structural stage separately requires positive progress; this
        # branch is retained here only as an explicit fail-closed enum.
        raise fail(f"{label}: no_inner_iteration cannot satisfy structural progress")
    _equal(report.get("relative_error_tolerance"), 1e-8,
           f"{label}/relative_error_tolerance")
    _equal(report.get("absolute_error_tolerance"), 1e-10,
           f"{label}/absolute_error_tolerance")
    _equal(report.get("error_tolerance"), 0.0, f"{label}/error_tolerance")
    _equal(report.get("initial_lambda"), 1e-5, f"{label}/initial_lambda")
    _equal(report.get("maximum_lambda"), 100000.0, f"{label}/maximum_lambda")
    _equal(report.get("lambda_factor"), 10.0, f"{label}/lambda_factor")
    _equal(report.get("lambda_lower_bound"), 0.0, f"{label}/lambda_lower_bound")
    _equal(report.get("lambda_upper_bound"), 100000.0, f"{label}/lambda_upper_bound")
    _equal(report.get("min_model_fidelity"), 0.001, f"{label}/min_model_fidelity")
    _equal(report.get("diagonal_damping"), False, f"{label}/diagonal_damping")
    _equal(report.get("use_fixed_lambda_factor"), True, f"{label}/use_fixed_lambda_factor")
    if stage == "main":
        _equal(report.get("linear_solver"), "MULTIFRONTAL_QR", f"{label}/linear_solver")
        _equal(report.get("elimination"), "EliminateQR", f"{label}/elimination")
    else:
        _equal(report.get("linear_solver"), "MULTIFRONTAL_CHOLESKY", f"{label}/linear_solver")
        _equal(report.get("elimination"), "EliminatePreferCholesky", f"{label}/elimination")
    _equal(report.get("ordering_type"), "COLAMD", f"{label}/ordering_type")
    _equal(report.get("explicit_ordering_present"), False,
           f"{label}/explicit_ordering_present")
    _true(report, "no_fallback", label)
    _true(report, "termination_trace_complete", label)
    _true(report, "configuration_valid", label)


def validate_structural_summary(route: str, summary: Mapping[str, Any]) -> None:
    """Validate one opaque structural summary and native Phase143 sidecar."""
    if route not in ROUTES:
        raise fail(f"summary: unknown route {route!r}")
    try:
        phase138 = _load_phase138_runner()
        phase138.validate_structural_summary(route, summary)
    except Phase143ContractError:
        raise
    except Exception as exc:
        raise fail(f"retained Phase138 gate failed: {exc}") from exc
    termination = _required(summary, "phase143_termination", f"summary/{route}")
    if not isinstance(termination, Mapping):
        raise fail(f"summary/{route}/phase143_termination: expected object")
    _equal(termination.get("schema_version"), NATIVE_TERMINATION_SCHEMA,
           f"summary/{route}/phase143_termination/schema_version")
    _equal(termination.get("authority"),
           "FGOResult.diagnostics.native_phase143_termination",
           f"summary/{route}/phase143_termination/authority")
    _true(termination, "no_solution_or_accuracy_fields", f"summary/{route}/phase143_termination")
    validate_termination_stage("gnss-first", _required(termination, "gnss_first",
                                                         f"summary/{route}/phase143_termination"))
    validate_termination_stage("main", _required(termination, "main",
                                                   f"summary/{route}/phase143_termination"))
    if any(key in termination for key in ("solution_rows", "solution_values", "coordinate_rows")):
        raise fail(f"summary/{route}/phase143_termination contains solution content")
    if any(key in summary for key in ("coordinate_rows", "solution_rows", "solution_values")):
        raise fail(f"summary/{route} contains solution content")


def validate_static_sources() -> dict[str, str]:
    result = {}
    for path_text, expected in SOURCE_SHA256.items():
        digest = static_sha256(ROOT / path_text, path_text)
        _equal(digest, expected, f"source/{path_text}/sha256")
        result[path_text] = digest
    digest = static_sha256(BINARY, "Phase143 target binary")
    _equal(digest, TARGET_BINARY_SHA256, "target binary sha256")
    result[relative(BINARY)] = digest
    native = (ROOT / "apps/native/gnss_fgo_imu_no_base.cpp").read_text(encoding="utf-8")
    backend = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text(encoding="utf-8")
    internal = (ROOT / "src/algorithms/fgo_gtsam_internal.hpp").read_text(encoding="utf-8")
    for needle in (
        PHASE143, "phase143_termination", NATIVE_TERMINATION_SCHEMA,
        "writePhase143TerminationDiagnostics",
    ):
        if needle not in native:
            raise fail(f"native Phase143 witness missing: {needle}")
    for needle in ("phase143EffectiveMaxIterations", "makePhase143TerminationDiagnostics",
                   "validatePhase143TerminationDiagnostics", "phase143_main_scope"):
        if needle not in backend + internal:
            raise fail(f"backend Phase143 witness missing: {needle}")
    return result


def launch_free_validation() -> dict[str, Any]:
    """Validate static qualification artifacts only; never launch a process."""
    audit_digest = static_sha256(AUDIT, "Phase143 structural audit")
    freeze_digest = static_sha256(FREEZE, "Phase143 structural freeze")
    design_digest = static_sha256(DESIGN_FREEZE, "Phase143 design freeze")
    _equal(audit_digest, AUDIT_SHA256, "audit sha256")
    _equal(freeze_digest, STRUCTURAL_FREEZE_SHA256, "structural freeze sha256")
    _equal(design_digest, DESIGN_FREEZE_SHA256, "design freeze sha256")
    freeze = read_json(FREEZE, "Phase143 structural freeze")
    manifest = read_json(MANIFEST, "Phase143 structural manifest")
    validate_freeze(freeze)
    validate_manifest(manifest)
    pre_raw = read_json(PRE_RAW, "Phase143 pre-raw seal")
    validate_pre_raw(pre_raw, manifest)
    source_hashes = validate_static_sources()
    _equal(manifest.get("source_pins"), source_hashes, "manifest/source_pins")
    return {
        "status": "launch-free-qualified",
        "phase": 143,
        "routes": list(ROUTES),
        "raw_execution_authorized": False,
        "solver_invocations": 0,
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_reads": 0,
        "truth_reads": 0,
        "solution_coordinate_reads": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_access": 0,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--print-commands", action="store_true",
                        help="print placeholder argv only; never launches inputs")
    args = parser.parse_args(argv)
    try:
        result = launch_free_validation()
        if args.print_commands:
            manifest = read_json(MANIFEST, "Phase143 structural manifest")
            for route in ROUTES:
                print(json.dumps({"route": route,
                                  "argv": manifest["command_snapshots"][route]},
                                 sort_keys=True))
        print(json.dumps(result, sort_keys=True))
    except Phase143ContractError as exc:
        print(f"PHASE143_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
