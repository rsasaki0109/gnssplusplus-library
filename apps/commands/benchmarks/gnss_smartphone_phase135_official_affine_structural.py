#!/usr/bin/env python3
"""Launch-free Phase135 structural contract and validator.

This module deliberately has no solver, raw-input, truth, MAT, Kaggle, or
network execution path.  It validates the pinned Phase135 recipe, emits only
placeholder argv snapshots, and validates an opaque structural summary supplied
by a later independently authorized runner.  The latter runner must materialize
raw inputs only after authorization and must never expose solution rows.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase135_official_affine_structural_freeze_v1.json"
)
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase135_official_affine_structural_manifest_v1.json"
)
RUNNER = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase135_official_affine_structural.py"
)
NATIVE_BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

FREEZE_SHA256 = (
    "005c90c8be2fa9cdfabdb777589064bc8ece22c5efa6872bf732cb329db872ce"
)
DESIGN_FREEZE_COMMIT = "bfafa340803f2dd1b5528c918b5be60f6e0e5c14"
IMPLEMENTATION_COMMIT = "48b13f098a75da87e52bcb15f66e3434c950baf7"
CORRECTION_COMMIT = "f9a1fc9403e707243a30d9e06aa8ea59493cdc5"
AUDIT_COMMIT = "01b7b68a37c668422c7ca396cfe56edab706f819"
TARGET_BINARY_SHA256 = (
    "bcd9a1a896c09248cd1ff3d1a4b1cd2eb9d1eebf0559073c4bad9158188c529f"
)

PHASE135_SELECTOR = "--native-phase135-official-affine-measurement-family"
PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = (
    "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
)
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = (
    "--native-phase128-glonass-provenance-parser-admission"
)
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
SCHEMA = "smartphone-r5-phase135-official-affine-structural-manifest.v1"
RAW_PLACEHOLDERS = {
    "--android-gnss": "__PHASE135_RAW_DEVICE_GNSS__",
    "--android-imu": "__PHASE135_RAW_DEVICE_IMU__",
    "--nav": "__PHASE135_RAW_BROADCAST_NAV__",
    "--native-base-rinex": "__PHASE135_RAW_BASE_RINEX__",
    "--native-base-rinex-sha256": "__PHASE135_RAW_BASE_SHA256__",
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
ON_SELECTORS = (PHASE118_SELECTOR, PHASE135_SELECTOR)
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


class Phase135ContractError(ValueError):
    """A Phase135 contract violation which must fail closed."""


def fail(message: str) -> Phase135ContractError:
    return Phase135ContractError(message)


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} must be an object")
    return value


def static_sha256(path: Path, label: str) -> str:
    """Hash a contract/source/binary artifact, never a payload path."""
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


def assert_zero_reads(accounting: Any, label: str) -> None:
    if not isinstance(accounting, Mapping):
        raise fail(f"{label}: missing read accounting")
    for key, value in accounting.items():
        if isinstance(value, bool):
            assert_equal(value, False, f"{label}/{key}")
        elif isinstance(value, int):
            assert_equal(value, 0, f"{label}/{key}")
        elif isinstance(value, str):
            assert_true(
                value in {"not-run", "read-only", "sealed-metadata-only"},
                f"{label}/{key}/marker",
            )
        else:
            raise fail(f"{label}/{key}: unsupported read marker")


def phase107_raw_base_recipe(freeze: Mapping[str, Any]) -> bool:
    recipe = freeze.get("recipe")
    if not isinstance(recipe, Mapping):
        return False
    return (
        recipe.get("phase107_raw_base_compensation") is True
        and recipe.get("phase107_raw_base_source_miss_mask") is True
        and recipe.get("phase107_preserve_additional_frequency_bands") is False
        and recipe.get("phase126_raw_base_source_complete") is False
        and recipe.get("phase127_glonass_channel_provenance") is False
        and recipe.get("phase128_glonass_provenance_parser_admission") is False
        and recipe.get("phase129_glonass_local_miss_mask") is False
        and recipe.get("phase131_canonical_correction_band_key") is False
    )


def validate_freeze(freeze: Mapping[str, Any]) -> None:
    assert_equal(freeze.get("phase"), 135, "freeze/phase")
    assert_equal(freeze.get("schema_version"),
                 "smartphone-r5-phase135-official-affine-structural-freeze.v1",
                 "freeze/schema")
    design = freeze.get("design_freeze")
    if not isinstance(design, Mapping):
        raise fail("freeze/design_freeze missing")
    assert_equal(design.get("commit"), DESIGN_FREEZE_COMMIT,
                 "freeze/design commit")
    assert_equal(design.get("implementation_commit"), IMPLEMENTATION_COMMIT,
                 "freeze/implementation commit")
    assert_equal(design.get("doppler_source_parity_audit_commit"), AUDIT_COMMIT,
                 "freeze/audit commit")
    assert_equal(design.get("doppler_source_parity_correction_commit"),
                 CORRECTION_COMMIT, "freeze/correction commit")
    recipe = freeze.get("recipe")
    if not isinstance(recipe, Mapping):
        raise fail("freeze/recipe missing")
    expected = {
        "phase135_official_affine_measurement_family": True,
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
        "phase99_main_multifrontal_qr": True,
        "corrected_undifferenced_doppler_factors": True,
        "skip_epochs": 0,
    }
    for key, value in expected.items():
        assert_equal(recipe.get(key), value, f"freeze/recipe/{key}")
    assert_equal(recipe.get("phase118_fixed_tdcp_sigma_m"), 0.03,
                 "freeze/recipe/fixed TDCP sigma")
    assert_equal(recipe.get("phase118_highway_huber_threshold_sigma"), 0.8,
                 "freeze/recipe/Highway Huber")
    assert_true(phase107_raw_base_recipe(freeze), "freeze/Phase107 recipe")
    auth = freeze.get("authorization_boundary")
    if not isinstance(auth, Mapping):
        raise fail("freeze/authorization_boundary missing")
    assert_equal(auth.get("pre_raw_authorized"), False,
                 "freeze/pre-raw authorization")
    assert_equal(auth.get("independent_raw_authorization_required"), True,
                 "freeze/raw authorization requirement")
    assert_equal(auth.get("one_shot_per_route"), True,
                 "freeze/one-shot policy")
    assert_equal(auth.get("truth_evaluation_authorized"), False,
                 "freeze/truth authorization")


def command_template(route: str) -> list[str]:
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    route_dir = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase135-official-affine-structural-v1"
    return [
        "build/apps/gnss_fgo_imu_no_base",
        "--dataset-id",
        route,
        "--android-gnss",
        RAW_PLACEHOLDERS["--android-gnss"],
        "--android-imu",
        RAW_PLACEHOLDERS["--android-imu"],
        "--nav",
        RAW_PLACEHOLDERS["--nav"],
        *REQUIRED_RECIPE_FLAGS,
        *ON_SELECTORS,
        "--native-base-rinex",
        RAW_PLACEHOLDERS["--native-base-rinex"],
        "--native-base-rinex-sha256",
        RAW_PLACEHOLDERS["--native-base-rinex-sha256"],
        "--out",
        f"{output_root}/{route_dir}/opaque_solution_output.csv",
        "--summary-json",
        f"{output_root}/{route_dir}/native_summary.json",
    ]


def validate_command(route: str, command: Any) -> None:
    if not isinstance(command, list) or any(
        not isinstance(item, str) for item in command
    ):
        raise fail(f"command/{route}: expected argv list")
    assert_equal(command, command_template(route), f"command/{route}")
    for token in command:
        if not token.startswith("--") and any(
            term in token.lower() for term in FORBIDDEN_PATH_TERMS
        ):
            # The opaque output filename is intentionally allowed; all input
            # placeholders must still be raw-only.
            if "opaque_solution_output.csv" not in token:
                raise fail(f"command/{route}: forbidden path token {token!r}")
    for selector in REQUIRED_RECIPE_FLAGS + ON_SELECTORS:
        assert_equal(command.count(selector), 1,
                     f"command/{route}/{selector}")
    for selector in OFF_SELECTORS:
        assert_equal(command.count(selector), 0,
                     f"command/{route}/{selector}")
    for flag, placeholder in RAW_PLACEHOLDERS.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}")
    summary = command[command.index("--summary-json") + 1]
    assert_true(summary.endswith("/native_summary.json"),
                f"command/{route}/native summary")
    assert_true("structural_summary" not in summary,
                f"command/{route}/summary separation")


def _required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def _true(mapping: Mapping[str, Any], key: str, label: str) -> None:
    assert_true(_required(mapping, key, label), f"{label}/{key}")


def _count(mapping: Mapping[str, Any], key: str, label: str) -> int:
    return nonnegative_int(_required(mapping, key, label), f"{label}/{key}")


def validate_structural_summary(
    route: str, summary: Mapping[str, Any]
) -> None:
    """Validate an opaque post-run structural summary, fail-closed."""
    if route not in ROUTES:
        raise fail(f"summary: unknown route {route!r}")
    assert_equal(summary.get("route"), route, f"summary/{route}/route")
    selectors = _required(summary, "selectors", f"summary/{route}")
    if not isinstance(selectors, Mapping):
        raise fail(f"summary/{route}/selectors: missing")
    selector_expectations = {
        "phase135_official_affine_measurement_family": True,
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
    for key, value in selector_expectations.items():
        assert_equal(selectors.get(key), value, f"summary/{route}/selector/{key}")

    phase135 = _required(summary, "phase135", f"summary/{route}")
    if not isinstance(phase135, Mapping):
        raise fail(f"summary/{route}/phase135: missing")
    _true(phase135, "enabled", f"summary/{route}/phase135")
    _true(phase135, "configuration_valid", f"summary/{route}/phase135")
    _true(phase135, "transactional", f"summary/{route}/phase135")
    _true(phase135, "fixed_initial_geometry", f"summary/{route}/phase135")
    _true(phase135, "finite_jacobians", f"summary/{route}/phase135")
    assert_equal(
        _required(phase135, "los_convention", f"summary/{route}/phase135"),
        "-e=(receiver-satellite)/range",
        f"summary/{route}/phase135/los convention",
    )
    _true(phase135, "single_sagnac_representation",
          f"summary/{route}/phase135")
    assert_equal(
        _required(phase135, "sagnac_evaluations",
                  f"summary/{route}/phase135"),
        _count(phase135, "geometry_rows", f"summary/{route}/phase135"),
        f"summary/{route}/phase135/sagnac conservation",
    )
    doppler = _required(phase135, "doppler", f"summary/{route}/phase135")
    if not isinstance(doppler, Mapping):
        raise fail(f"summary/{route}/phase135/doppler: missing")
    for key in (
        "official_rate_residual",
        "receiver_velocity_included",
        "explicit_sagnac_included",
        "source_provenance_complete",
        "source_provenance_finite",
    ):
        _true(doppler, key, f"summary/{route}/phase135/doppler")
    assert_equal(
        _required(doppler, "los_convention",
                  f"summary/{route}/phase135/doppler"),
        "-e=(receiver-satellite)/range",
        f"summary/{route}/phase135/doppler/los",
    )
    for family in ("pseudorange", "doppler", "ordinary_tdcp"):
        family_data = _required(phase135, family,
                                f"summary/{route}/phase135")
        if not isinstance(family_data, Mapping):
            raise fail(f"summary/{route}/phase135/{family}: missing")
        admitted = _count(family_data, "admitted_rows",
                          f"summary/{route}/phase135/{family}")
        inserted = _count(family_data, "affine_factors_inserted",
                          f"summary/{route}/phase135/{family}")
        assert_true(admitted > 0,
                    f"summary/{route}/phase135/{family}/admitted_rows")
        assert_equal(inserted, admitted,
                     f"summary/{route}/phase135/{family}/conservation")
        _true(family_data, "key_order_exact",
              f"summary/{route}/phase135/{family}")
        _true(family_data, "finite_values",
              f"summary/{route}/phase135/{family}")
    legacy = _required(phase135, "legacy_factor_counts",
                       f"summary/{route}/phase135")
    if not isinstance(legacy, Mapping):
        raise fail(f"summary/{route}/phase135/legacy_factor_counts: missing")
    for key, value in legacy.items():
        assert_equal(value, 0, f"summary/{route}/phase135/legacy/{key}")
    bridge = _required(phase135, "pose3_x_bridge",
                       f"summary/{route}/phase135")
    if not isinstance(bridge, Mapping):
        raise fail(f"summary/{route}/phase135/pose3_x_bridge: missing")
    assert_true(_count(bridge, "count",
                       f"summary/{route}/phase135/pose3_x_bridge") > 0,
                f"summary/{route}/phase135/pose3_x_bridge/count")
    _true(bridge, "keys_exact", f"summary/{route}/phase135/pose3_x_bridge")

    base = _required(summary, "raw_base", f"summary/{route}")
    if not isinstance(base, Mapping):
        raise fail(f"summary/{route}/raw_base: missing")
    for key in (
        "phase107_recipe",
        "applied_exactly_once",
        "source_miss_conservation",
        "no_raw_or_zero_fallback",
    ):
        _true(base, key, f"summary/{route}/raw_base")

    clock = _required(summary, "clock", f"summary/{route}")
    if not isinstance(clock, Mapping):
        raise fail(f"summary/{route}/clock: missing")
    assert_equal(clock.get("c_units"), "metres",
                 f"summary/{route}/clock/C units")
    assert_equal(clock.get("d_units"), "metres/second",
                 f"summary/{route}/clock/D units")
    _true(clock, "c7_mapping_exact", f"summary/{route}/clock")
    _true(clock, "d_full_finite_exact_alignment", f"summary/{route}/clock")

    solver = _required(summary, "solver", f"summary/{route}")
    if not isinstance(solver, Mapping):
        raise fail(f"summary/{route}/solver: missing")
    assert_equal(solver.get("linear_solver"), "MULTIFRONTAL_QR",
                 f"summary/{route}/solver/linear_solver")
    assert_equal(solver.get("elimination"), "EliminateQR",
                 f"summary/{route}/solver/elimination")
    for stage in ("gnss_first", "main"):
        stage_data = _required(solver, stage, f"summary/{route}/solver")
        if not isinstance(stage_data, Mapping):
            raise fail(f"summary/{route}/solver/{stage}: missing")
        assert_true(_count(stage_data, "accepted_iterations",
                           f"summary/{route}/solver/{stage}") > 0,
                    f"summary/{route}/solver/{stage}/accepted_iterations")
        initial = finite_number(_required(
            stage_data, "initial_cost", f"summary/{route}/solver/{stage}"),
            f"summary/{route}/solver/{stage}/initial_cost")
        final = finite_number(_required(
            stage_data, "final_cost", f"summary/{route}/solver/{stage}"),
            f"summary/{route}/solver/{stage}/final_cost")
        if not final < initial:
            raise fail(f"summary/{route}/solver/{stage}: cost did not decrease")
        _true(stage_data, "no_fallback", f"summary/{route}/solver/{stage}")

    output = _required(summary, "output", f"summary/{route}")
    if not isinstance(output, Mapping):
        raise fail(f"summary/{route}/output: missing")
    _true(output, "finite", f"summary/{route}/output")
    _true(output, "earth_valid", f"summary/{route}/output")
    _true(output, "expected_epoch_coverage", f"summary/{route}/output")
    assert_equal(output.get("pixel5_offset_applications"), 1,
                 f"summary/{route}/output/Pixel5 offset")
    _true(output, "opaque_solution_seal", f"summary/{route}/output")
    assert_true("coordinate_rows" not in output,
                f"summary/{route}/output/coordinate rows")

    reads = _required(summary, "read_accounting", f"summary/{route}")
    if not isinstance(reads, Mapping):
        raise fail(f"summary/{route}/read_accounting: missing")
    for key, value in reads.items():
        lowered = key.lower()
        if any(term in lowered for term in
               ("truth", "mat", "pdc", "precomputed", "kaggle", "accuracy")):
            assert_equal(value, 0, f"summary/{route}/read/{key}")
    assert_equal(summary.get("fallback"), False, f"summary/{route}/fallback")
    assert_equal(summary.get("rerun"), False, f"summary/{route}/rerun")


def validate_manifest(manifest: Mapping[str, Any]) -> None:
    assert_equal(manifest.get("schema_version"), SCHEMA, "manifest/schema")
    assert_equal(manifest.get("phase"), 135, "manifest/phase")
    assert_equal(manifest.get("status"), "launch-free", "manifest/status")
    assert_equal(manifest.get("freeze_sha256"), FREEZE_SHA256,
                 "manifest/freeze sha")
    assert_equal(manifest.get("design_freeze_commit"), DESIGN_FREEZE_COMMIT,
                 "manifest/design commit")
    assert_equal(manifest.get("implementation_commit"), IMPLEMENTATION_COMMIT,
                 "manifest/implementation commit")
    assert_equal(manifest.get("correction_commit"), CORRECTION_COMMIT,
                 "manifest/correction commit")
    assert_equal(manifest.get("target_binary_sha256"), TARGET_BINARY_SHA256,
                 "manifest/binary sha")
    routes = manifest.get("routes")
    if not isinstance(routes, list):
        raise fail("manifest/routes missing")
    assert_equal(routes, list(ROUTES), "manifest/route order")
    recipe = manifest.get("recipe")
    if not isinstance(recipe, Mapping):
        raise fail("manifest/recipe missing")
    assert_equal(recipe.get("on_selectors"),
                 [PHASE118_SELECTOR, PHASE135_SELECTOR],
                 "manifest/on selectors")
    assert_equal(recipe.get("off_selectors"), list(OFF_SELECTORS),
                 "manifest/off selectors")
    assert_equal(recipe.get("phase107_raw_base"), True,
                 "manifest/Phase107 raw base")
    assert_equal(recipe.get("phase126_134_compound"), False,
                 "manifest/Phase126-134 compound")
    commands = manifest.get("command_snapshots")
    if not isinstance(commands, Mapping):
        raise fail("manifest/command_snapshots missing")
    for route in ROUTES:
        validate_command(route, commands.get(route))
    accounting = manifest.get("pre_raw_read_accounting")
    assert_zero_reads(accounting, "manifest/pre_raw_read_accounting")
    policy = manifest.get("policy")
    if not isinstance(policy, Mapping):
        raise fail("manifest/policy missing")
    for key in (
        "truth_used",
        "mat_used",
        "pdc_used",
        "precomputed_coordinates_used",
        "accuracy_evaluation",
        "kaggle_access",
        "rerun",
        "fallback",
        "solution_publication",
    ):
        assert_equal(policy.get(key), False, f"manifest/policy/{key}")
    assert_equal(policy.get("raw_execution_authorized"), False,
                 "manifest/policy/raw authorization")


def launch_free_validation() -> dict[str, Any]:
    """Read and validate only the static freeze/manifest contract."""
    freeze = read_json(FREEZE, "Phase135 structural freeze")
    manifest = read_json(MANIFEST, "Phase135 structural manifest")
    assert_equal(static_sha256(FREEZE, "Phase135 freeze"), FREEZE_SHA256,
                 "freeze/file sha")
    validate_freeze(freeze)
    validate_manifest(manifest)
    assert_equal(static_sha256(RUNNER, "Phase135 launch-free runner"),
                 manifest.get("runner_source_sha256"),
                 "runner/source sha")
    return {
        "status": "launch-free-qualified",
        "phase": 135,
        "routes": list(ROUTES),
        "raw_execution_authorized": False,
        "solver_invocations": 0,
        "truth_reads": 0,
        "solution_coordinate_reads": 0,
        "mat_pdc_precomputed_reads": 0,
        "kaggle_access": 0,
    }


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
            manifest = read_json(MANIFEST, "Phase135 structural manifest")
            for route in ROUTES:
                print(json.dumps(
                    {"route": route, "argv": manifest["command_snapshots"][route]},
                    sort_keys=True,
                ))
        print(json.dumps(result, sort_keys=True))
    except Phase135ContractError as exc:
        print(f"PHASE135_FAIL_CLOSED: {exc}")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
