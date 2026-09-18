#!/usr/bin/env python3
"""Launch-free Phase138 affine-TDCP structural contract.

Only tracked source, static contract JSON, placeholder argv, and synthetic
in-memory summaries are handled here.  This module never opens raw GNSS/IMU,
navigation, or base payloads, never reads solution/truth rows, and never
launches the native solver.  A later runner needs a separate authorization
before it can materialize the two raw-only routes.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_affine_tdcp_structural_audit_v1.md"
)
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_affine_tdcp_structural_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase138_affine_tdcp_structural_manifest_v1.json"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural.py"
)
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase138_affine_tdcp_structural.py"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"


AUDIT_COMMIT = "739b940dc870e21de42deba6ba911c17a93f0526"
AUDIT_SHA256 = "5cd012e87b3d2d2fecd5b0d5897b1252d99a979166108de244e92aaf8b4545fe"
FREEZE_COMMIT = "f1f5fef2ae0ccdcbae94be86e6af813b65ec499b"
FREEZE_SHA256 = "b5616d60815a68d7b0a978355b47ade80a222426e991b5bc506a81a5ad5f9aa3"
DESIGN_FREEZE_COMMIT = "49be79247b4d82bf7a64c7b63e542c5a6dfff2c0"
IMPLEMENTATION_COMMIT = "c5783d7e323b0c4593958a4e210284cf9f9fc726"
PHASE135_CORRECTION_COMMIT = "f9a1fc9403e7072435a30d9e06aa8ea59493cdc5"
TARGET_BINARY_SHA256 = (
    "b2254284328b52bb3aa4ebdba731e795fea2e6a32197478cb2423828d3e43b12"
)

SOURCE_SHA256 = {
    "apps/native/gnss_fgo_imu_no_base.cpp":
        "6b77c8ac90f9cbff073c838d3aeaae832634269ca45027591eb2cf689a63397b",
    "include/libgnss++/algorithms/fgo.hpp":
        "d2794573f15a1b4ac569ecb97869c9a5bfcbd519ae471472aaaa0a3d6f167ce6",
    "include/libgnss++/algorithms/fgo_config.hpp":
        "eed0d551ebfc8eec73ff3abb6cfe70ac3a9f7669476d661c24596b05baf8c9ff",
    "include/libgnss++/algorithms/tdcp_contract.hpp":
        "f4423e1944fd58f28ad779fa6a6ba87919e358360cdf1ddda59b0af20a954d57",
    "src/algorithms/fgo_gtsam_backend.cpp":
        "2d28e4825e49143919a7c6254940e4b14a53883b5d9d3928a09e8730e3309fcf",
}

PHASE135_SELECTOR = "--native-phase135-official-affine-measurement-family"
PHASE138_SELECTOR = "--native-phase138-affine-tdcp-anchor-range-constant"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE130_SELECTOR = "--native-phase130-shared-ledger-key-local-support"
PHASE131_SELECTOR = "--native-phase131-canonical-correction-band-key"
ADDITIONAL_SELECTOR = (
    "--native-base-pseudorange-preserve-additional-frequency-bands"
)

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
ROUTE_LABELS = {ROUTES[0]: "MTV-A", ROUTES[1]: "LAX-T"}
SCHEMA = "smartphone-r5-phase138-affine-tdcp-structural-manifest.v1"

RAW_PLACEHOLDERS = {
    "--android-gnss": "__PHASE138_RAW_DEVICE_GNSS__",
    "--android-imu": "__PHASE138_RAW_DEVICE_IMU__",
    "--nav": "__PHASE138_RAW_BROADCAST_NAV__",
    "--native-base-rinex": "__PHASE138_RAW_BASE_RINEX__",
    "--native-base-rinex-sha256": "__PHASE138_RAW_BASE_SHA256__",
}
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "truth",
    "ground_truth",
    "precomputed",
    "coordinate",
    "pdc",
    "kaggle",
    "token",
)
REQUIRED_RECIPE_FLAGS = (
    "--all-epochs",
    "--android-raw-utc-keys",
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
    # This is an existing native algorithm switch, not a PDC input path.
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
ON_SELECTORS = (PHASE135_SELECTOR, PHASE138_SELECTOR, PHASE118_SELECTOR)
OFF_SELECTORS = (
    PHASE117_SELECTOR,
    PHASE120_SELECTOR,
    PHASE126_SELECTOR,
    PHASE127_SELECTOR,
    PHASE128_SELECTOR,
    PHASE129_SELECTOR,
    PHASE130_SELECTOR,
    PHASE131_SELECTOR,
    ADDITIONAL_SELECTOR,
)


class Phase138ContractError(ValueError):
    """A Phase138 contract violation which must fail closed."""


def fail(message: str) -> Phase138ContractError:
    return Phase138ContractError(message)


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
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


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value: Any, label: str) -> None:
    if value is not True:
        raise fail(f"{label}: expected true")


def nonnegative_int(value: Any, label: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or value < 0:
        raise fail(f"{label}: expected nonnegative integer")
    return value


def finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise fail(f"{label}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        raise fail(f"{label}: expected finite number")
    return result


def assert_zero_reads(value: Any, label: str) -> None:
    if not isinstance(value, Mapping):
        raise fail(f"{label}: missing read accounting")
    for key, item in value.items():
        if isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, int):
            assert_equal(item, 0, f"{label}/{key}")
        elif isinstance(item, str):
            if item not in {"not-run", "read-only", "sealed-metadata-only"}:
                raise fail(f"{label}/{key}: invalid marker {item!r}")
        else:
            raise fail(f"{label}/{key}: unsupported read marker")


def command_template(route: str) -> list[str]:
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    route_dir = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase138-affine-tdcp-structural-v1"
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
    assert_equal(command, command_template(route), f"command/{route}")
    for token in command:
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            if "opaque_solution_output.csv" not in token:
                raise fail(f"command/{route}: forbidden path token {token!r}")
    for selector in REQUIRED_RECIPE_FLAGS + ON_SELECTORS:
        assert_equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in OFF_SELECTORS:
        assert_equal(command.count(selector), 0, f"command/{route}/{selector}")
    for flag, placeholder in RAW_PLACEHOLDERS.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")
    summary_path = command[command.index("--summary-json") + 1]
    assert_true(summary_path.endswith("/native_summary.json"),
                f"command/{route}/native summary path")
    assert_true("structural_summary" not in summary_path,
                f"command/{route}/summary separation")


def validate_freeze(freeze: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": "smartphone-r5-phase138-affine-tdcp-structural-freeze.v1",
        "phase": 138,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-structural-contract",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, Mapping):
        raise fail("freeze/audit missing")
    assert_equal(audit.get("path"), relative(AUDIT), "freeze/audit/path")
    assert_equal(audit.get("commit"), AUDIT_COMMIT, "freeze/audit/commit")
    assert_equal(audit.get("sha256"), AUDIT_SHA256, "freeze/audit/sha256")
    design = freeze.get("design_freeze")
    if not isinstance(design, Mapping):
        raise fail("freeze/design_freeze missing")
    assert_equal(design.get("commit"), DESIGN_FREEZE_COMMIT,
                 "freeze/design commit")
    assert_equal(design.get("implementation_commit"), IMPLEMENTATION_COMMIT,
                 "freeze/implementation commit")
    assert_equal(design.get("phase135_corrected_source_commit"),
                 PHASE135_CORRECTION_COMMIT, "freeze/Phase135 correction")

    candidate = freeze.get("candidate")
    if not isinstance(candidate, Mapping):
        raise fail("freeze/candidate missing")
    for key, expected in {
        "count": 1,
        "id": "phase138-affine-tdcp-anchor-range-constant-v1",
        "selector": PHASE138_SELECTOR,
        "default_off": True,
        "partial_selector_set_allowed": False,
        "raw_execution_authorized": False,
        "truth_evaluation_authorized": False,
        "solution_publication_authorized": False,
    }.items():
        assert_equal(candidate.get(key), expected, f"freeze/candidate/{key}")
    assert_equal(candidate.get("requires"),
                 [PHASE135_SELECTOR, PHASE118_SELECTOR],
                 "freeze/candidate/requires")
    assert_equal(candidate.get("measurement_equation"),
                 "tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)",
                 "freeze/candidate/equation")
    exact = candidate.get("exactly_once_invariants")
    if not isinstance(exact, Mapping):
        raise fail("freeze/candidate/exactly_once_invariants missing")
    for key, expected in {
        "range_constants_validated": "ordinary_tdcp_admitted_rows",
        "measurements_adjusted": "ordinary_tdcp_admitted_rows",
        "affine_tdcp_factor_count": "ordinary_tdcp_admitted_rows",
        "adjustment_application_passes": 1,
        "adjusted_exactly_once": True,
        "factor_count_unchanged": True,
        "same_endpoint_epoch_and_satellite_state": True,
        "single_sagnac_representation": True,
        "finite_adjusted_measurements": True,
    }.items():
        assert_equal(exact.get(key), expected, f"freeze/exactly_once/{key}")
    recipe = freeze.get("recipe")
    if not isinstance(recipe, Mapping):
        raise fail("freeze/recipe missing")
    for key, expected in {
        "on_selectors": [PHASE135_SELECTOR, PHASE138_SELECTOR, PHASE118_SELECTOR],
        "off_selectors": list(OFF_SELECTORS),
        "phase107_raw_base": True,
        "phase126_134_compound": False,
        "route_type": "Highway",
        "runs_per_route": 1,
    }.items():
        assert_equal(recipe.get(key), expected, f"freeze/recipe/{key}")
    assert_equal(recipe.get("route_order"), list(ROUTES),
                 "freeze/recipe/route order")
    fixed = candidate.get("fixed")
    if not isinstance(fixed, Mapping):
        raise fail("freeze/candidate/fixed missing")
    assert_equal(fixed.get("phase118_tdcp_sigma_m"), 0.03,
                 "freeze/candidate/fixed TDCP sigma")
    assert_equal(fixed.get("phase118_highway_huber_k"), 0.5,
                 "freeze/candidate/fixed Highway Huber k")
    gates = freeze.get("structural_gates")
    if not isinstance(gates, Mapping):
        raise fail("freeze/structural_gates missing")
    for key in (
        "selector_isolation",
        "phase135_all_affine_and_legacy_zero",
        "phase138_range_verified_anchor_applied_affine_count_equal",
        "phase138_same_endpoint_satellite_state_and_sagnac",
        "phase138_exactly_once_and_transactional",
        "raw_base_exactly_once_and_miss_conserved",
        "c7_d_exact_full_finite_alignment",
        "qr_solver",
        "gnss_first_and_main_accepted_iterations_positive",
        "gnss_first_and_main_finite_strict_cost_decrease",
        "finite_earth_valid_expected_epoch_coverage",
        "pixel5_offset_exactly_once",
        "no_fallback_retry_rerun_or_solution_rows",
    ):
        assert_true(gates.get(key), f"freeze/structural_gates/{key}")
    routes = freeze.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("freeze/routes order changed")
    for item in routes:
        assert_equal(item.get("planned_solver_invocations"), 1,
                     f"freeze/{item.get('dataset_id')}/invocations")
    accounting = freeze.get("read_accounting_before_authorization")
    assert_zero_reads(accounting, "freeze/read_accounting_before_authorization")
    boundary = freeze.get("authorization_boundary")
    if not isinstance(boundary, Mapping):
        raise fail("freeze/authorization_boundary missing")
    for key, expected in {
        "independent_raw_authorization_commit_required": True,
        "materialize_inputs_after_authorization_only": True,
        "raw_result_commit_separate": True,
        "truth_accuracy_authorization": False,
        "solution_publication": False,
        "rerun_fallback_repair_sweep": False,
    }.items():
        assert_equal(boundary.get(key), expected, f"freeze/boundary/{key}")


def validate_static_sources() -> dict[str, str]:
    result: dict[str, str] = {}
    for relative_path, expected in SOURCE_SHA256.items():
        path = ROOT / relative_path
        digest = static_sha256(path, relative_path)
        assert_equal(digest, expected, f"source/{relative_path}/sha256")
        result[relative_path] = digest
    digest = static_sha256(BINARY, "Phase138 target binary")
    assert_equal(digest, TARGET_BINARY_SHA256, "target binary sha256")
    result[relative(BINARY)] = digest
    # These checks prove that the structural contract is tied to the actual
    # endpoint/state/Sagnac implementation, rather than to a caller count.
    backend = (ROOT / "src/algorithms/fgo_gtsam_backend.cpp").read_text(encoding="utf-8")
    required = (
        "phase138_tdcp_range_constants_validated",
        "phase138_tdcp_measurements_adjusted",
        "phase138_adjusted_exactly_once",
        "Phase138AffineTdcpMeasurement",
        "applyPhase138AffineTdcpAnchorRangeConstant",
        "factor.current_source_satellite_position_ecef",
        "factor.previous_source_satellite_position_ecef",
        "phase135GeometryAt",
        "current_geometry",
        "single-Sagnac",
    )
    for needle in required:
        if needle not in backend:
            raise fail(f"source geometry witness missing: {needle}")
    return result


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    for key, expected in {
        "schema_version": SCHEMA,
        "phase": 138,
        "status": "launch-free",
        "freeze_sha256": FREEZE_SHA256,
        "design_freeze_commit": DESIGN_FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "phase135_corrected_source_commit": PHASE135_CORRECTION_COMMIT,
        "target_binary_sha256": TARGET_BINARY_SHA256,
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    assert_equal(manifest.get("freeze_path"), relative(FREEZE), "manifest/freeze path")
    assert_equal(manifest.get("audit_commit"), AUDIT_COMMIT, "manifest/audit commit")
    assert_equal(manifest.get("structural_freeze_commit"), FREEZE_COMMIT,
                 "manifest/structural freeze commit")
    assert_equal(manifest.get("routes"), list(ROUTES), "manifest/route order")
    recipe = manifest.get("recipe")
    if not isinstance(recipe, Mapping):
        raise fail("manifest/recipe missing")
    assert_equal(recipe.get("on_selectors"), list(ON_SELECTORS),
                 "manifest/on selectors")
    assert_equal(recipe.get("off_selectors"), list(OFF_SELECTORS),
                 "manifest/off selectors")
    for key, expected in {
        "phase107_raw_base": True,
        "phase126_134_compound": False,
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "phase99_main_linear_solver": "MULTIFRONTAL_QR",
        "phase99_main_elimination": "EliminateQR",
    }.items():
        assert_equal(recipe.get(key), expected, f"manifest/recipe/{key}")
    commands = manifest.get("command_snapshots")
    if not isinstance(commands, Mapping):
        raise fail("manifest/command_snapshots missing")
    for route in ROUTES:
        validate_command(route, commands.get(route))
    policy = manifest.get("policy")
    if not isinstance(policy, Mapping):
        raise fail("manifest/policy missing")
    for key in (
        "raw_execution_authorized",
        "truth_used",
        "mat_used",
        "pdc_used",
        "precomputed_coordinates_used",
        "accuracy_evaluation",
        "kaggle_access",
        "solution_publication",
        "rerun",
        "fallback",
        "repair",
        "sweep",
    ):
        assert_equal(policy.get(key), False, f"manifest/policy/{key}")
    assert_equal(policy.get("opaque_solution_only"), True,
                 "manifest/policy/opaque solution")
    assert_zero_reads(manifest.get("pre_raw_read_accounting"),
                      "manifest/pre_raw_read_accounting")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, Mapping):
        raise fail("manifest/artifacts missing")
    for key, path in (("runner", RUNNER), ("focused_tests", FOCUSED_TESTS)):
        item = artifacts.get(key)
        if not isinstance(item, Mapping):
            raise fail(f"manifest/artifacts/{key} missing")
        assert_equal(item.get("path"), relative(path),
                     f"manifest/artifacts/{key}/path")
        digest = item.get("sha256")
        if not isinstance(digest, str) or len(digest) != 64:
            raise fail(f"manifest/artifacts/{key}/sha256 missing")
        assert_equal(static_sha256(path, f"manifest/artifacts/{key}"), digest,
                     f"manifest/artifacts/{key}/sha256")
    structural = manifest.get("structural_summary_schema")
    if not isinstance(structural, Mapping):
        raise fail("manifest/structural_summary_schema missing")
    assert_equal(structural.get("solution_coordinate_fields_forbidden"), True,
                 "manifest/structural summary solution policy")


def launch_free_validation() -> dict[str, Any]:
    """Validate static artifacts only; do not materialize or launch anything."""
    freeze = read_json(FREEZE, "Phase138 structural freeze")
    manifest = read_json(MANIFEST, "Phase138 structural manifest")
    assert_equal(static_sha256(AUDIT, "Phase138 audit"), AUDIT_SHA256,
                 "audit/file sha")
    assert_equal(static_sha256(FREEZE, "Phase138 freeze"), FREEZE_SHA256,
                 "freeze/file sha")
    validate_freeze(freeze)
    validate_manifest(manifest)
    source_hashes = validate_static_sources()
    assert_equal(manifest.get("source_pins"), source_hashes,
                 "manifest/source pins")
    return {
        "status": "launch-free-qualified",
        "phase": 138,
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


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _true(mapping: Mapping[str, Any], key: str, label: str) -> None:
    assert_true(_required(mapping, key, label), f"{label}/{key}")


def _count(mapping: Mapping[str, Any], key: str, label: str) -> int:
    return nonnegative_int(_required(mapping, key, label), f"{label}/{key}")


def _validate_family(family: Mapping[str, Any], label: str) -> int:
    admitted = _count(family, "admitted_rows", label)
    inserted = _count(family, "affine_factors_inserted", label)
    if admitted <= 0:
        raise fail(f"{label}/admitted_rows must be positive")
    assert_equal(inserted, admitted, f"{label}/admission conservation")
    _true(family, "key_order_exact", label)
    _true(family, "finite_values", label)
    _true(family, "source_geometry_same_path", label)
    return admitted


def validate_structural_summary(route: str, summary: Mapping[str, Any]) -> None:
    """Validate one opaque authorized-run summary, fail closed."""
    if route not in ROUTES:
        raise fail(f"summary: unknown route {route!r}")
    assert_equal(summary.get("route"), route, f"summary/{route}/route")
    selectors = _required(summary, "selectors", f"summary/{route}")
    if not isinstance(selectors, Mapping):
        raise fail(f"summary/{route}/selectors missing")
    expected_selectors = {
        "phase135_official_affine_measurement_family": True,
        "phase138_affine_tdcp_anchor_range_constant": True,
        "phase118_official_tdcp_huber_k": True,
        "phase117_dynamic_tdcp_sigma": False,
        "phase120_official_tdcp_resl_atmosphere_cancellation": False,
        "phase126_raw_base_source_complete": False,
        "phase127_glonass_channel_provenance": False,
        "phase128_glonass_provenance_parser_admission": False,
        "phase129_glonass_local_miss_mask": False,
        "phase130_shared_ledger_key_local_support": False,
        "phase131_canonical_correction_band_key": False,
        "phase107_raw_base_compensation": True,
        "phase107_raw_base_source_miss_mask": True,
        "phase107_preserve_additional_frequency_bands": False,
    }
    for key, expected in expected_selectors.items():
        assert_equal(selectors.get(key), expected,
                     f"summary/{route}/selector/{key}")

    phase135 = _required(summary, "phase135", f"summary/{route}")
    if not isinstance(phase135, Mapping):
        raise fail(f"summary/{route}/phase135 missing")
    for key in ("enabled", "configuration_valid", "transactional",
                "fixed_initial_geometry", "finite_jacobians",
                "single_sagnac_representation"):
        _true(phase135, key, f"summary/{route}/phase135")
    assert_equal(_required(phase135, "los_convention", f"summary/{route}/phase135"),
                 "-e=(receiver-satellite)/range",
                 f"summary/{route}/phase135/los convention")
    assert_equal(_count(phase135, "sagnac_evaluations", f"summary/{route}/phase135"),
                 _count(phase135, "geometry_rows", f"summary/{route}/phase135"),
                 f"summary/{route}/phase135/Sagnac conservation")
    families = {}
    for name in ("pseudorange", "doppler", "ordinary_tdcp"):
        value = _required(phase135, name, f"summary/{route}/phase135")
        if not isinstance(value, Mapping):
            raise fail(f"summary/{route}/phase135/{name} missing")
        families[name] = _validate_family(value,
                                          f"summary/{route}/phase135/{name}")
    legacy = _required(phase135, "legacy_factor_counts",
                       f"summary/{route}/phase135")
    if not isinstance(legacy, Mapping):
        raise fail(f"summary/{route}/phase135/legacy_factor_counts missing")
    for key, value in legacy.items():
        assert_equal(value, 0, f"summary/{route}/phase135/legacy/{key}")
    bridge = _required(phase135, "pose3_x_bridge", f"summary/{route}/phase135")
    if not isinstance(bridge, Mapping):
        raise fail(f"summary/{route}/phase135/pose3_x_bridge missing")
    if _count(bridge, "count", f"summary/{route}/phase135/pose3_x_bridge") <= 0:
        raise fail(f"summary/{route}/phase135/Pose3-X bridge is empty")
    _true(bridge, "keys_exact", f"summary/{route}/phase135/pose3_x_bridge")

    phase138 = _required(summary, "phase138", f"summary/{route}")
    if not isinstance(phase138, Mapping):
        raise fail(f"summary/{route}/phase138 missing")
    for key in ("enabled", "phase135_dependency_satisfied", "configuration_valid",
                "adjusted_exactly_once", "factor_count_unchanged",
                "phase118_atmosphere_sigma_huber_unchanged",
                "single_sagnac_representation",
                "same_endpoint_epoch_and_satellite_state",
                "same_satellite_state",
                "finite_adjusted_measurements",
                "no_raw_or_zero_fallback",
                "transactional"):
        _true(phase138, key, f"summary/{route}/phase138")
    assert_equal(_required(phase138, "measurement_equation",
                           f"summary/{route}/phase138"),
                 "tdcp_phase138 = tdcp_native - (rho_current_initial - rho_previous_initial)",
                 f"summary/{route}/phase138/equation")
    assert_equal(_required(phase138, "geometry_representation",
                           f"summary/{route}/phase138"),
                 "RTKLIB-geodist-single-Sagnac-fixed-initial-endpoints",
                 f"summary/{route}/phase138/geometry")
    tdcp_count = families["ordinary_tdcp"]
    for key in ("range_constants_validated", "tdcp_measurements_adjusted",
                "affine_tdcp_factor_count"):
        assert_equal(_count(phase138, key, f"summary/{route}/phase138"),
                     tdcp_count, f"summary/{route}/phase138/{key}/count")
    assert_equal(_count(phase138, "adjustment_application_passes",
                        f"summary/{route}/phase138"), 1,
                 f"summary/{route}/phase138/application passes")
    assert_equal(_count(phase138, "legacy_tdcp_factor_count",
                        f"summary/{route}/phase138"), 0,
                 f"summary/{route}/phase138/legacy count")

    base = _required(summary, "raw_base", f"summary/{route}")
    if not isinstance(base, Mapping):
        raise fail(f"summary/{route}/raw_base missing")
    for key in ("phase107_recipe", "applied_exactly_once",
                "source_miss_conservation", "no_raw_or_zero_fallback"):
        _true(base, key, f"summary/{route}/raw_base")

    clock = _required(summary, "clock", f"summary/{route}")
    if not isinstance(clock, Mapping):
        raise fail(f"summary/{route}/clock missing")
    assert_equal(clock.get("c_units"), "metres", f"summary/{route}/clock/C units")
    assert_equal(clock.get("d_units"), "metres/second",
                 f"summary/{route}/clock/D units")
    _true(clock, "c7_mapping_exact", f"summary/{route}/clock")
    _true(clock, "d_full_finite_exact_alignment", f"summary/{route}/clock")

    solver = _required(summary, "solver", f"summary/{route}")
    if not isinstance(solver, Mapping):
        raise fail(f"summary/{route}/solver missing")
    assert_equal(solver.get("linear_solver"), "MULTIFRONTAL_QR",
                 f"summary/{route}/solver/linear_solver")
    assert_equal(solver.get("elimination"), "EliminateQR",
                 f"summary/{route}/solver/elimination")
    for stage in ("gnss_first", "main"):
        data = _required(solver, stage, f"summary/{route}/solver")
        if not isinstance(data, Mapping):
            raise fail(f"summary/{route}/solver/{stage} missing")
        if _count(data, "accepted_iterations",
                  f"summary/{route}/solver/{stage}") <= 0:
            raise fail(f"summary/{route}/solver/{stage}/accepted_iterations is zero")
        initial = finite_number(_required(data, "initial_cost",
                                          f"summary/{route}/solver/{stage}"),
                                f"summary/{route}/solver/{stage}/initial_cost")
        final = finite_number(_required(data, "final_cost",
                                        f"summary/{route}/solver/{stage}"),
                              f"summary/{route}/solver/{stage}/final_cost")
        if not final < initial:
            raise fail(f"summary/{route}/solver/{stage}: cost did not decrease")
        _true(data, "no_fallback", f"summary/{route}/solver/{stage}")

    output = _required(summary, "output", f"summary/{route}")
    if not isinstance(output, Mapping):
        raise fail(f"summary/{route}/output missing")
    for key in ("finite", "earth_valid", "expected_epoch_coverage",
                "opaque_solution_seal"):
        _true(output, key, f"summary/{route}/output")
    assert_equal(output.get("pixel5_offset_applications"), 1,
                 f"summary/{route}/output/Pixel5 offset")
    if "coordinate_rows" in output or "solution_rows" in output:
        raise fail(f"summary/{route}/output contains solution coordinates")

    reads = _required(summary, "read_accounting", f"summary/{route}")
    if not isinstance(reads, Mapping):
        raise fail(f"summary/{route}/read_accounting missing")
    for key, value in reads.items():
        lowered = key.lower()
        if any(term in lowered for term in
               ("truth", "mat", "pdc", "precomputed", "kaggle", "accuracy")):
            assert_equal(value, 0, f"summary/{route}/read/{key}")
        elif isinstance(value, int):
            nonnegative_int(value, f"summary/{route}/read/{key}")
        elif isinstance(value, bool):
            assert_equal(value, False, f"summary/{route}/read/{key}")
        else:
            raise fail(f"summary/{route}/read/{key}: unsupported marker")
    assert_equal(summary.get("fallback"), False, f"summary/{route}/fallback")
    assert_equal(summary.get("rerun"), False, f"summary/{route}/rerun")


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--print-commands",
        action="store_true",
        help="print placeholder argv snapshots; never materializes inputs",
    )
    args = parser.parse_args(argv)
    try:
        result = launch_free_validation()
        if args.print_commands:
            manifest = read_json(MANIFEST, "Phase138 structural manifest")
            for route in ROUTES:
                print(json.dumps({
                    "route": route,
                    "argv": manifest["command_snapshots"][route],
                }, sort_keys=True))
        print(json.dumps(result, sort_keys=True))
    except Phase138ContractError as exc:
        print(f"PHASE138_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
