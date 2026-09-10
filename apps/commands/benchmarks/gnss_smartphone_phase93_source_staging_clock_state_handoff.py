#!/usr/bin/env python3
"""Fail-closed, pre-raw verifier for the sealed Phase93 Luna Max plan.

The verifier reads only source files and sealed records.  It validates the
single opt-in candidate, exact four-route x1 command matrix, source/binary
pins, structural gates, and zero-read boundary.  It has no native process
launch path and never opens or hashes a raw GNSS, IMU, navigation, truth, or
coordinate artifact.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]

PHASE93_AUDIT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_audit_v1.md"
)
PHASE93_AUDIT_SHA256 = (
    "ed0f11f404f5db1f3379af5723cffed825508a9e877208ad84b89e6d84bd4112"
)
PHASE93_SOURCE_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_freeze_v1.json"
)
PHASE93_SOURCE_FREEZE_SHA256 = (
    "34ca3a2c6beecdb912aed81c469eca8a45d32e80bad69e9fc5fa12ea1b49ab88"
)
PHASE93_SOURCE_FREEZE_COMMIT = "9336344ec5e5fdaacae1bdb4e0a2237adb9b0941"
PHASE80_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
)
PHASE80_FREEZE_SHA256 = (
    "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"
)
PHASE80_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase80_source_exact_direct_p_quality_structural_manifest_v1.json"
)
PHASE80_MANIFEST_SHA256 = (
    "6d20541560b726ed2409cd05902e5105874ac72e113af0c3812f029130df310f"
)
PHASE81_RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_result_v1.json"
)
PHASE81_RESULT_SHA256 = (
    "f5809f173c3e346aec775ca6dd152de5436eb68ea348d3fe90dffc7f82153b14"
)
PHASE91_RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase91_source_clock_c0d_gnss_first_raw_drift_d_initializer_result_v1.json"
)
PHASE91_RESULT_SHA256 = (
    "9929e285b58b5d9a65350ba1a3d497c736968bb896d72dbe5fb014534b6d58c8"
)
PHASE92_RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_structural_result_v1.json"
)
PHASE92_RESULT_SHA256 = (
    "8488306b3ec61d0360418de73fa1596271597b6971d77de676fcd279d7e1b01c"
)
PHASE92_EXECUTION_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_execution_freeze_v1.json"
)
PHASE92_EXECUTION_FREEZE_SHA256 = (
    "c07ac587ebcfde6391de0b8a824687316de854442166a93daadb416d4e9b74d8"
)
PHASE92_EXECUTION_MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase92_source_meter_clock_state_parity_execution_manifest_v1.json"
)
PHASE92_EXECUTION_MANIFEST_SHA256 = (
    "d6370b03f5d9dc48e8eb8da5667f06252caff444a2d8998cdac4380aea8f29a3"
)

EXECUTION_FREEZE = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_execution_freeze_v1.json"
)
EXECUTION_FREEZE_SHA256 = "6893bf21496b2ecf3afbe43ad94fc4cde81a84a2ac03b47fb08ef30a13047f55"
MANIFEST = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase93_source_staging_clock_state_handoff_execution_manifest_v1.json"
)
EVALUATOR = Path(__file__).resolve()
FOCUSED_TESTS = ROOT / (
    "tests/test_smartphone_phase93_source_staging_clock_state_handoff.py"
)
TESTS_CMAKE = ROOT / "tests/CMakeLists.txt"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

IMPLEMENTATION_COMMIT = "2613adb12fba80c468f8c6945ba360a126129f58"
SELECTOR = "--native-source-clock-c0d-gnss-first-meter-state-handoff"
CLOCK_FLAG = "--native-source-clock-c0d-factor"
METER_FLAG = "--native-source-clock-c0d-meter-state-parity"
ACTIVE_FLAG = "--native-source-clock-c0d-active-solve-diagnostic"
RAW_DRIFT_FLAG = "--native-source-clock-c0d-gnss-first-raw-drift-d-initializer"
DIRECT_FLAG = "--native-source-direct-observable-quality"
NO_BRIDGE_FLAG = "--native-pdc-imu-tdcp-no-bridge"
RAW_UTC_FLAG = "--android-raw-utc-keys"
COMPATIBILITY_FLAGS = (
    "--android-raw-clock-only",
    "--android-utc-wall-clock-fallback",
)
RAW_INPUT_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
DOMAIN_ROWS = {
    ROUTES[0]: 2158,
    ROUTES[1]: 3139,
    ROUTES[2]: 1465,
    ROUTES[3]: 1101,
}
OUTPUT_ROOT = "output/smartphone-r5/phase93-source-staging-clock-state-handoff-v1/"
RAW_ROOT = "raw/phase93/"

FORBIDDEN_FLAGS = (
    "--obs",
    "--native-gnss-first-velocity-only-handoff",
    "--native-direct-doppler-wls-handoff",
    "--native-pdc-state-bridge",
    "--native-upstream-quality",
    "--native-base-rinex",
    "--native-base-pseudorange-compensation",
    "--native-base-pseudorange-source-miss-mask",
    "--native-base-pseudorange-preserve-additional-frequency-bands",
)
FORBIDDEN_PATH_TERMS = (
    ".mat",
    "ground_truth",
    "validation",
    "holdout",
    "kaggle",
    "token",
    "base.rinex",
    "coordinate",
    "truth",
)
REQUIRED_FLAGS = (
    "--android-gnss <frozen raw device_gnss.csv>",
    "--android-imu <frozen raw device_imu.csv>",
    "--nav <frozen broadcast brdc.nav>",
    "--dataset-id <one of the four frozen route IDs>",
    "--all-epochs",
    RAW_UTC_FLAG,
    *COMPATIBILITY_FLAGS,
    NO_BRIDGE_FLAG,
    DIRECT_FLAG,
    CLOCK_FLAG,
    METER_FLAG,
    ACTIVE_FLAG,
    SELECTOR,
)

SOURCE_PINS = {
    "apps/native/gnss_fgo_imu_no_base.cpp": "7a47bb8bf715a9be3aeed0b29b391746b6753a1a2dea99d7235294669b5b92a2",
    "include/libgnss++/algorithms/fgo.hpp": "56d4dffb567cbc0b9fd96e2cbeef6dae080aadc510601145396de0cf39db4af3",
    "include/libgnss++/algorithms/fgo_config.hpp": "40c5db753ac7cf0e629866d3b704ef30725c407ca7d601a5d15029c136ffe2ad",
    "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp": "43a4be7e553407fe9417a8548b33f74870cd339c496239836fa589febe490237",
    "include/libgnss++/core/observation.hpp": "7a81b4f1c7d36b4af2ed8a11c0f1edcbb998591a22a4d1406d75aeb43641f11b",
    "src/algorithms/fgo.cpp": "a807945fddc116038539b8bd122e2562c07f4b92f919a7e66acaeaf036f3332b",
    "src/algorithms/fgo_gtsam_backend.cpp": "cafefea8c0a982a58dd860468f433176bab055690a3f41c996882adc5c57500a",
    "src/algorithms/fgo_gtsam_internal.hpp": "2e2b6ac4e9e01d1c82b423774c53512d8217e4f04b3a2aeb00b99ea018aeaea8",
    "src/algorithms/fgo_problems.cpp": "e7607ce8f0fe27fd382a01b5b6f147b027b7bf51453e920eae458c86ddac0c17",
    "src/io/android_raw_gnss.cpp": "a020ce900db6a9d3a994cc3adb3dcebbe84cbf67c1ff9793991b7a75039529c5",
    "tests/test_android_raw_gnss.cpp": "40a77cbf37468f39fb9b1ef30b8ab94a05900cfc6ab789b7b66fcdae947f6bd2",
    "tests/test_fgo.cpp": "60ede7bdd22ddc6e0db3f494edea523b5dbe48f1e6119afda1ba65046e805b2d",
    "tests/test_fgo_gtsam_backend.cpp": "901330e37b9b1b5047bb9f6e5789c8f1b293b7f1cfcc1df35b76f3534af71658",
}
PHASE93_CHANGED_PATHS = (
    "apps/native/gnss_fgo_imu_no_base.cpp",
    "include/libgnss++/algorithms/fgo.hpp",
    "include/libgnss++/algorithms/fgo_config.hpp",
    "src/algorithms/fgo.cpp",
    "src/algorithms/fgo_gtsam_backend.cpp",
    "src/algorithms/fgo_gtsam_internal.hpp",
    "tests/test_fgo_gtsam_backend.cpp",
)


class Phase93PreRawError(ValueError):
    """Raised when the sealed Phase93 pre-raw contract fails."""


def fail(message: str) -> Phase93PreRawError:
    return Phase93PreRawError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_forbidden_path(path: Path | str) -> None:
    lowered = str(path).lower()
    if any(term in lowered for term in FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden artifact path: {path}")


def _read_bytes(path: Path, label: str, expected_sha256: str | None = None) -> bytes:
    _reject_forbidden_path(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if expected_sha256 is not None and hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _sha256(path: Path, label: str) -> str:
    return hashlib.sha256(_read_bytes(path, label)).hexdigest()


def _read_json(path: Path, label: str, expected_sha256: str | None = None) -> dict[str, Any]:
    try:
        value = json.loads(_read_bytes(path, label, expected_sha256).decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def _assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label} changed: expected {expected!r}, got {actual!r}")


def _assert_zero_accounting(accounting: dict[str, Any], label: str) -> None:
    for key in (
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "base_rinex_reads",
        "native_solver_invocations",
        "native_reruns",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "validation_holdout_reads",
        "kaggle_or_token_access",
    ):
        _assert_equal(accounting.get(key), 0, f"{label}/{key}")
    _assert_equal(accounting.get("accuracy_scored"), False, f"{label}/accuracy_scored")
    _assert_equal(accounting.get("route_score_selection"), False, f"{label}/route_score_selection")


def _source_text(path_text: str, label: str) -> str:
    return _read_bytes(ROOT / path_text, label).decode("utf-8")


def _require_source_terms() -> None:
    sources = {
        "entrypoint": _source_text("apps/native/gnss_fgo_imu_no_base.cpp", "entrypoint source"),
        "config": _source_text("include/libgnss++/algorithms/fgo_config.hpp", "FGO config source"),
        "result": _source_text("include/libgnss++/algorithms/fgo.hpp", "FGO result source"),
        "processor": _source_text("src/algorithms/fgo.cpp", "FGO processor source"),
        "backend": _source_text("src/algorithms/fgo_gtsam_backend.cpp", "GTSAM backend source"),
        "internal": _source_text("src/algorithms/fgo_gtsam_internal.hpp", "GTSAM internal source"),
        "initializer": _source_text(
            "include/libgnss++/algorithms/source_clock_c0d_initializer.hpp",
            "retained-key initializer source",
        ),
        "builder": _source_text("src/algorithms/fgo_problems.cpp", "problem builder source"),
        "parser": _source_text("src/io/android_raw_gnss.cpp", "Android GNSS parser source"),
        "tests": _source_text("tests/test_fgo_gtsam_backend.cpp", "GTSAM focused tests"),
    }
    required = {
        "entrypoint": (
            SELECTOR,
            "gnss_first_config.use_native_source_clock_c0d_factor = true;",
            "gnss_first_config.use_native_source_clock_c0d_meter_state_parity = true;",
            "gnss_first_config.use_native_source_clock_c0d_active_solve_diagnostic =\n                true;",
            "raw_drift_mps.reserve(android_gnss.observations.epochs.size())",
            "validatePhase93OptimizedDExport",
            "gnss_first_result.epoch_clock_drift_mps",
            "problem.native_source_clock_c0d_gnss_first_d_handoff_mps",
            "SPEED_OF_LIGHT",
            "earthValidEcef",
        ),
        "config": (
            "bool use_native_source_clock_c0d_gnss_first_meter_state_handoff = false;",
        ),
        "result": ("std::vector<double> epoch_clock_drift_mps;",),
        "processor": (
            "use_native_source_clock_c0d_gnss_first_meter_state_handoff",
        ),
        "backend": (
            "if (use_pose3 && !use_native_source_clock_c0d_meter_state)",
            "graph.emplace_shared<gtsam::CarrierPhaseFactor>",
            "native_source_clock_c0d_gnss_first_d_handoff_mps",
            "native_source_clock_c0d_gnss_first_meter_state_handoff",
        ),
        "internal": (
            "clockStateScale",
            "clockStateTerm",
            "publicClockSeconds",
            "dt_s_ / (2.0 * clockStateScale(meter_state_))",
        ),
        "initializer": (
            "validateRetainedRawDAlignment",
            "raw_source_index",
            "raw_utc_time_millis",
            "exact (not nearest) lookup",
        ),
        "builder": (
            "seed.raw_source_index = epoch.raw_source_index",
            "seed.raw_utc_time_millis = epoch.raw_utc_time_millis",
        ),
        "parser": ("raw_source_index", "raw_utc_time_millis"),
        "tests": (
            "FGOGtsamSourceClockC0DPhase93Test",
            "GnssFirstPoint3VelocityUsesMeterC0DAndExportsOptimizedDrift",
            "MainGraphRequiresOptimizedDVectorAndDoesNotUseRawFallback",
            "RetainedRawAlignmentUsesExplicitSourceKeysAndAllowsFilteredRows",
            "CandidateConfigurationIsDefaultOffAndConflictsFailClosed",
        ),
    }
    for name, terms in required.items():
        for term in terms:
            if term not in sources[name]:
                raise fail(f"Phase93 source contract term missing: {name}: {term}")


def verify_authority() -> None:
    _read_bytes(PHASE93_AUDIT, "Phase93 audit", PHASE93_AUDIT_SHA256)
    freeze = _read_json(
        PHASE93_SOURCE_FREEZE,
        "Phase93 source staging freeze",
        PHASE93_SOURCE_FREEZE_SHA256,
    )
    _assert_equal(freeze.get("phase"), 93, "Phase93 source freeze/phase")
    _assert_equal(
        freeze.get("status"),
        "frozen-before-phase93-implementation",
        "Phase93 source freeze/status",
    )
    _assert_equal(freeze.get("execution_label"), "Luna Max", "Phase93 source freeze/execution_label")
    candidate = freeze.get("candidate", {})
    _assert_equal(candidate.get("exactly_one"), True, "Phase93 source freeze/candidate/exactly_one")
    _assert_equal(candidate.get("id"), "phase93_source_meter_c0d_gnss_first_retained_clock_state_handoff", "Phase93 source freeze/candidate/id")
    _assert_equal(candidate.get("selector"), SELECTOR, "Phase93 source freeze/candidate/selector")
    _assert_equal(candidate.get("default_off"), True, "Phase93 source freeze/candidate/default_off")
    _assert_equal(candidate.get("implementation_authorized"), True, "Phase93 source freeze/candidate/implementation_authorized")
    _assert_equal(candidate.get("raw_execution_authorized"), False, "Phase93 source freeze/candidate/raw_execution_authorized")
    _assert_equal(candidate.get("accuracy_scored"), False, "Phase93 source freeze/candidate/accuracy_scored")
    boundary = freeze.get("execution_boundary", {})
    _assert_equal(boundary.get("raw_execution_authorized"), False, "Phase93 source freeze/execution_boundary/raw_execution_authorized")
    accounting = freeze.get("read_accounting_at_freeze", {})
    for key in (
        "native_solver_invocations",
        "raw_device_gnss_reads",
        "raw_device_imu_reads",
        "broadcast_navigation_reads",
        "base_rinex_reads",
        "truth_reads",
        "mat_reads_or_generated",
        "precomputed_coordinate_reads",
        "validation_holdout_reads",
        "kaggle_or_token_access",
        "accuracy_calculations",
        "raw_input_hash_reads",
    ):
        _assert_equal(accounting.get(key), 0, f"Phase93 source freeze/read_accounting/{key}")


def verify_execution_freeze() -> dict[str, Any]:
    verify_authority()
    freeze = _read_json(EXECUTION_FREEZE, "Phase93 execution freeze")
    _assert_equal(
        freeze.get("schema_version"),
        "smartphone-r5-phase93-source-staging-clock-state-handoff-execution-freeze.v1",
        "execution freeze/schema_version",
    )
    _assert_equal(freeze.get("phase"), 93, "execution freeze/phase")
    _assert_equal(freeze.get("status"), "frozen-before-phase93-raw-execution", "execution freeze/status")
    _assert_equal(freeze.get("execution_label"), "Luna Max", "execution freeze/execution_label")
    authority = freeze.get("authority", {})
    _assert_equal(authority.get("phase93_source_staging_freeze", {}).get("sha256"), PHASE93_SOURCE_FREEZE_SHA256, "execution freeze/source freeze pin")
    _assert_equal(authority.get("phase93_source_staging_freeze", {}).get("commit"), PHASE93_SOURCE_FREEZE_COMMIT, "execution freeze/source freeze commit")
    _assert_equal(authority.get("phase93_audit", {}).get("sha256"), PHASE93_AUDIT_SHA256, "execution freeze/audit pin")
    implementation = freeze.get("implementation", {})
    _assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT, "execution freeze/implementation commit")
    _assert_equal(implementation.get("candidate_count"), 1, "execution freeze/candidate count")
    _assert_equal(implementation.get("legacy_default_unchanged"), True, "execution freeze/legacy default")
    _assert_equal(implementation.get("sigma_or_lm_tuning"), False, "execution freeze/tuning")
    expected_paths = sorted(PHASE93_CHANGED_PATHS)
    _assert_equal(sorted(implementation.get("allowed_phase93_delta_paths", [])), expected_paths, "execution freeze/allowed source delta")

    candidate = freeze.get("candidate", {})
    for key, expected in {
        "id": "phase93_source_meter_c0d_gnss_first_retained_clock_state_handoff",
        "selector": SELECTOR,
        "exactly_one": True,
        "default_off": True,
        "raw_only": True,
        "diagnostic_only": True,
        "controls": 0,
        "runs_per_route": 1,
        "accuracy_scoring": False,
        "route_score_selection": False,
        "no_pdc": True,
        "no_direct_doppler_wls": True,
        "no_velocity_only_handoff": True,
        "no_external_or_precomputed_coordinates": True,
        "no_truth_mat_kaggle_token": True,
    }.items():
        _assert_equal(candidate.get(key), expected, f"execution freeze/candidate/{key}")
    gnss_first = candidate.get("gnss_first", {})
    for key, expected in {
        "graph": "Point3 position plus explicit velocity states",
        "use_pose3_state": False,
        "use_imu": False,
        "source_meter_c0d": True,
        "clock_C_and_isb_unit": "metres",
        "drift_D_unit": "metres_per_second",
        "dt_unit": "seconds",
        "residual_unit": "metres",
        "ordinary_sigma_m": 0.1,
        "equation": "(C2-C1)-((D1+D2)*dt/2)",
        "jacobian_order": ["C1", "C2", "D1", "D2"],
        "jacobian": "[-1,+1,-dt/2,-dt/2]",
        "raw_D_initializer": "exact retained EpochSeed.receiver_clock_drift_mps by source keys",
        "optimizer_max_iterations": 1000,
        "no_sigma_or_lm_tuning": True,
    }.items():
        _assert_equal(gnss_first.get(key), expected, f"execution freeze/candidate/gnss_first/{key}")
    handoff = candidate.get("result_and_handoff", {})
    for key, expected in {
        "optimized_D_result_field": "FGOResult.epoch_clock_drift_mps",
        "optimized_D_order": "retained GNSS-first source order",
        "optimized_D_exact_length": True,
        "optimized_D_finite_full_coverage": True,
        "optimized_D_fallback": "none",
        "clock_C_main_conversion": "C_LIGHT exactly once to metres",
        "clock_D_source": "same-run optimized FGOResult.epoch_clock_drift_mps[i]",
        "main_D_initializer": "optimized GNSS-first D only",
        "exact_retained_key_alignment": True,
        "raw_or_zero_fallback": False,
        "interpolation_or_padding": False,
    }.items():
        _assert_equal(handoff.get(key), expected, f"execution freeze/candidate/result_and_handoff/{key}")

    routes = freeze.get("routes_and_domain", {})
    _assert_equal(routes.get("route_order"), list(ROUTES), "execution freeze/route order")
    _assert_equal(routes.get("domain_rows"), DOMAIN_ROWS, "execution freeze/domain rows")
    for key, expected in {"exactly_one_run_each": True, "route_count": 4, "controls": 0}.items():
        _assert_equal(routes.get(key), expected, f"execution freeze/routes/{key}")

    gates = freeze.get("structural_gates", {})
    for stage in ("gnss_first", "main"):
        stage_gates = gates.get(stage, {})
        for key, expected in {
            "accepted_outer_iterations_min_exclusive": 0,
            "finite_initial_and_final_cost": True,
            "strict_final_cost_lt_initial_cost": True,
            "finite_earth_valid_position_output": True,
            "finite_velocity_output": True,
            "finite_optimized_C_and_D_output": True,
        }.items():
            _assert_equal(stage_gates.get(key), expected, f"execution freeze/gates/{stage}/{key}")
    conditioning = gates.get("conditioning_telemetry", {})
    _assert_equal(conditioning.get("required"), True, "execution freeze/conditioning required")
    _assert_equal(conditioning.get("finite"), True, "execution freeze/conditioning finite")
    _assert_equal(conditioning.get("no_tuning"), True, "execution freeze/conditioning tuning")
    _assert_equal(
        gates.get("handoff", {}).get("retained_key_fields"),
        ["raw_source_index", "raw_utc_time_millis", "GNSS_week_tow"],
        "execution freeze/handoff key fields",
    )

    evidence = freeze.get("qualification_evidence", {}).get("full_cpp_suite", {})
    for key, expected in {
        "registered_tests": 1101,
        "passed": 1043,
        "skipped": 58,
        "failed": 0,
        "phase93_related_tests_passed": 2,
        "phase93_related_tests_failed": 0,
        "raw_execution": False,
        "truth_mat_kaggle_base_or_precomputed_coordinates": False,
    }.items():
        _assert_equal(evidence.get(key), expected, f"execution freeze/qualification/{key}")

    boundary = freeze.get("execution_boundary", {})
    for key, expected in {
        "before_raw_execution": True,
        "raw_execution_authorized": False,
        "no_raw_execution_performed_at_freeze": True,
        "native_solver_invocations_at_freeze": 0,
        "native_route_rerun_performed": False,
        "accuracy_scoring": False,
        "accuracy_or_submission_release": False,
        "stop_before_accuracy_or_submission": True,
    }.items():
        _assert_equal(boundary.get(key), expected, f"execution freeze/boundary/{key}")
    _assert_zero_accounting(freeze.get("read_accounting_at_freeze", {}), "execution freeze/read accounting")
    return freeze


def _verify_source_pins(manifest: dict[str, Any]) -> None:
    implementation = manifest.get("implementation", {})
    _assert_equal(implementation.get("commit"), IMPLEMENTATION_COMMIT, "manifest/implementation/commit")
    _assert_equal(implementation.get("source_hashes"), SOURCE_PINS, "manifest/implementation/source_hashes")
    for path_text, expected in SOURCE_PINS.items():
        if _sha256(ROOT / path_text, f"implementation source {path_text}") != expected:
            raise fail(f"implementation source hash changed: {path_text}")
    binary = implementation.get("binary", {})
    _assert_equal(binary.get("path"), relative(BINARY), "manifest/binary/path")
    expected_binary_sha = binary.get("sha256")
    if not isinstance(expected_binary_sha, str) or len(expected_binary_sha) != 64:
        raise fail("manifest/binary/sha256 is not sealed")
    if _sha256(BINARY, "implementation binary") != expected_binary_sha:
        raise fail("implementation binary hash changed")


def _raw_input_path(route: str, name: str) -> str:
    return f"{RAW_ROOT}{route}/{name}"


def _validate_raw_input_plan(route: str, raw_inputs: Any) -> None:
    if not isinstance(raw_inputs, dict) or tuple(raw_inputs) != RAW_INPUT_NAMES:
        raise fail(f"raw input set is not exactly GNSS+IMU+nav: {route}")
    for name in RAW_INPUT_NAMES:
        item = raw_inputs.get(name)
        if not isinstance(item, dict):
            raise fail(f"raw input pin is not an object: {route}/{name}")
        path_text = item.get("path")
        if not isinstance(path_text, str) or not path_text.startswith(RAW_ROOT):
            raise fail(f"raw input path is not a pre-raw placeholder: {route}/{name}")
        if Path(path_text).is_absolute() or Path(path_text).name != name:
            raise fail(f"raw input path/name mismatch: {route}/{name}")
        if path_text != _raw_input_path(route, name):
            raise fail(f"raw input route path changed: {route}/{name}")
        _reject_forbidden_path(path_text)
        for key, expected in {
            "sha256": None,
            "sha256_available": False,
            "read_at_manifest_creation": False,
        }.items():
            _assert_equal(item.get(key), expected, f"raw input {route}/{name}/{key}")


def _validate_command(route: str, command: Any, raw_inputs: dict[str, Any]) -> None:
    if not isinstance(command, list) or not command or command[0] != relative(BINARY):
        raise fail(f"native command binary changed: {route}")
    if any(not isinstance(token, str) for token in command):
        raise fail(f"native command contains a non-string token: {route}")
    for token in command:
        if any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"native command contains forbidden artifact token: {route}: {token}")
        if token in FORBIDDEN_FLAGS:
            raise fail(f"native command contains forbidden flag: {route}: {token}")
    expected_flags = (
        RAW_UTC_FLAG,
        *COMPATIBILITY_FLAGS,
        NO_BRIDGE_FLAG,
        DIRECT_FLAG,
        CLOCK_FLAG,
        METER_FLAG,
        ACTIVE_FLAG,
        SELECTOR,
    )
    for flag in expected_flags:
        if command.count(flag) != 1:
            raise fail(f"native command flag multiplicity failed: {route}/{flag}")
    if RAW_DRIFT_FLAG in command:
        raise fail(f"Phase91 raw-D-only initializer leaked into Phase93 command: {route}")
    if command.count("--dataset-id") != 1 or command[command.index("--dataset-id") + 1] != route:
        raise fail(f"native command dataset identity changed: {route}")
    for flag, name in (
        ("--android-gnss", "device_gnss.csv"),
        ("--android-imu", "device_imu.csv"),
        ("--nav", "brdc.nav"),
    ):
        if command.count(flag) != 1:
            raise fail(f"native command input flag multiplicity failed: {route}/{flag}")
        value = command[command.index(flag) + 1]
        if value != raw_inputs[name]["path"]:
            raise fail(f"native command raw path does not match manifest: {route}/{name}")
    for flag in ("--all-epochs",):
        if command.count(flag) != 1:
            raise fail(f"native command required flag missing: {route}/{flag}")
    for flag in ("--out", "--summary-json"):
        if command.count(flag) != 1:
            raise fail(f"native command output flag missing: {route}/{flag}")
        value = command[command.index(flag) + 1]
        if not value.startswith(OUTPUT_ROOT):
            raise fail(f"native command output path escaped Phase93 root: {route}")


def verify_manifest() -> dict[str, Any]:
    verify_execution_freeze()
    manifest = _read_json(MANIFEST, "Phase93 execution manifest")
    _assert_equal(
        manifest.get("schema_version"),
        "smartphone-r5-phase93-source-staging-clock-state-handoff-execution-manifest.v1",
        "manifest/schema_version",
    )
    _assert_equal(manifest.get("phase"), 93, "manifest/phase")
    _assert_equal(manifest.get("status"), "sealed-before-phase93-raw-execution", "manifest/status")
    _assert_equal(manifest.get("execution_label"), "Luna Max", "manifest/execution_label")
    freeze_ref = manifest.get("freeze", {})
    _assert_equal(freeze_ref.get("path"), relative(EXECUTION_FREEZE), "manifest/freeze/path")
    _assert_equal(freeze_ref.get("sha256"), EXECUTION_FREEZE_SHA256, "manifest/freeze/sha256")
    diff_audit = manifest.get("source_diff_audit", {})
    _assert_equal(diff_audit.get("base_phase93_freeze_commit"), PHASE93_SOURCE_FREEZE_COMMIT, "manifest/source diff/base freeze")
    _assert_equal(diff_audit.get("implementation_commit"), IMPLEMENTATION_COMMIT, "manifest/source diff/implementation")
    _assert_equal(diff_audit.get("diff_check_clean"), True, "manifest/source diff/check")
    _assert_equal(sorted(diff_audit.get("changed_paths_exactly", [])), sorted(PHASE93_CHANGED_PATHS), "manifest/source diff/changed paths")
    _assert_equal(diff_audit.get("paths_outside_phase93_boundary"), [], "manifest/source diff/outside paths")
    _assert_equal(diff_audit.get("candidate_count"), 1, "manifest/source diff/candidate count")
    _assert_equal(diff_audit.get("candidate_tuning"), False, "manifest/source diff/tuning")
    _verify_source_pins(manifest)
    for key, path, label in (
        ("evaluator", EVALUATOR, "pre-raw evaluator"),
        ("focused_tests", FOCUSED_TESTS, "focused pre-raw tests"),
        ("cmake", TESTS_CMAKE, "tests CMake"),
    ):
        pin = manifest.get(key, {})
        _assert_equal(pin.get("path"), relative(path), f"manifest/{key}/path")
        _assert_equal(pin.get("sha256"), _sha256(path, label), f"manifest/{key}/sha256")

    candidate = manifest.get("candidate", {})
    for key, expected in {
        "id": "phase93_source_meter_c0d_gnss_first_retained_clock_state_handoff",
        "selector": SELECTOR,
        "candidate_count": 1,
        "raw_only": True,
        "source_aligned": True,
        "diagnostic_only": True,
        "default_off": True,
        "controls": 0,
        "runs_per_route": 1,
        "accuracy_scoring": False,
        "route_score_selection": False,
        "no_pdc": True,
        "no_direct_doppler_wls": True,
        "no_velocity_only_handoff": True,
        "no_external_or_precomputed_coordinates": True,
    }.items():
        _assert_equal(candidate.get(key), expected, f"manifest/candidate/{key}")
    _assert_equal(candidate.get("required_flags"), list(REQUIRED_FLAGS), "manifest/candidate/required_flags")
    _assert_equal(
        candidate.get("forbidden_flags"),
        [*FORBIDDEN_FLAGS, RAW_DRIFT_FLAG, "any truth, MAT, Kaggle/token, base, result-file, or precomputed-coordinate input"],
        "manifest/candidate/forbidden_flags",
    )

    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for item in routes:
        route = item.get("dataset_id")
        if route not in DOMAIN_ROWS:
            raise fail(f"manifest has an unknown route: {route}")
        _assert_equal(item.get("runs"), 1, f"manifest/route/{route}/runs")
        _assert_equal(item.get("domain_rows"), DOMAIN_ROWS[route], f"manifest/route/{route}/domain_rows")
        _assert_equal(item.get("diagnostic_only"), True, f"manifest/route/{route}/diagnostic_only")
        _validate_raw_input_plan(route, item.get("raw_inputs"))
        _validate_command(route, item.get("command"), item["raw_inputs"])
        planned = item.get("planned_output", {})
        for key in ("structural_output", "summary"):
            value = planned.get(key)
            if not isinstance(value, str) or not value.startswith(OUTPUT_ROOT):
                raise fail(f"manifest planned output escaped Phase93 root: {route}/{key}")

    matrix = manifest.get("matrix", {})
    for key, expected in {
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "candidate_runs_total": 4,
        "control_runs_per_route": 0,
        "native_invocations_planned": 4,
        "raw_device_gnss_reads_planned": 4,
        "raw_device_imu_reads_planned": 4,
        "broadcast_nav_reads_planned": 4,
        "base_rinex_reads_planned": 0,
        "truth_reads_planned": 0,
        "mat_reads_or_generated_planned": 0,
        "precomputed_coordinate_reads_planned": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
    }.items():
        _assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    gates = manifest.get("structural_gates", {})
    _assert_equal(gates.get("required_before_release"), True, "manifest/structural_gates/required_before_release")
    for stage in ("gnss_first", "main"):
        stage_gates = gates.get(stage, {})
        for key in (
            "accepted_outer_iterations_min_exclusive",
            "strict_final_cost_lt_initial_cost",
            "finite_optimized_C_and_D_output",
            "expected_output_coverage",
        ):
            if key not in stage_gates:
                raise fail(f"manifest structural gate missing: {stage}/{key}")
    conditioning = gates.get("conditioning_telemetry", {})
    _assert_equal(conditioning.get("required"), True, "manifest/conditioning required")
    _assert_equal(conditioning.get("finite"), True, "manifest/conditioning finite")

    qualification = manifest.get("qualification_evidence", {})
    _assert_equal(qualification.get("full_cpp_suite", {}).get("failed"), 0, "manifest/full C++ failures")
    _assert_equal(qualification.get("full_cpp_suite", {}).get("phase93_related_tests_failed"), 0, "manifest/Phase93 C++ failures")
    _assert_equal(qualification.get("skip_classification", {}).get("phase93_caused"), False, "manifest/skip classification")

    authorization = manifest.get("execution_authorization", {})
    for key, expected in {
        "before_raw_execution": True,
        "raw_execution_authorized": False,
        "native_route_rerun_performed": False,
        "stop_before_accuracy_or_submission": True,
        "accuracy_or_submission_release": False,
    }.items():
        _assert_equal(authorization.get(key), expected, f"manifest/authorization/{key}")
    _assert_zero_accounting(manifest.get("read_accounting_at_manifest_creation", {}), "manifest/read accounting")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    freeze = verify_execution_freeze()
    manifest = verify_manifest()
    source = _read_bytes(EVALUATOR, "pre-raw evaluator source").decode("utf-8")
    for forbidden in (
        "import " + "sub" + "process",
        "sub" + "process.",
        "Popen" + "(",
        "os." + "system",
        "read_" + "raw",
    ):
        if forbidden in source:
            raise fail(f"pre-raw evaluator contains a process/raw-launch token: {forbidden}")
    _require_source_terms()
    return {
        "status": "pre-raw-verified",
        "phase": 93,
        "execution_label": "Luna Max",
        "candidate_count": 1,
        "routes": 4,
        "runs_per_route": 1,
        "raw_execution_authorized": False,
        "raw_reads": 0,
        "native_solver_invocations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_scored": False,
        "route_score_selection": False,
        "freeze_sha256": EXECUTION_FREEZE_SHA256,
        "manifest_sha256": _sha256(MANIFEST, "Phase93 execution manifest"),
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "freeze_status": freeze["status"],
        "manifest_status": manifest["status"],
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args(argv)
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("one of --verify-freeze, --verify-manifest, or --verify-pre-raw is required")
    try:
        if args.verify_freeze:
            verify_execution_freeze()
        if args.verify_manifest:
            verify_manifest()
        if args.verify_pre_raw:
            print(json.dumps(verify_pre_raw(), sort_keys=True))
        return 0
    except Phase93PreRawError as exc:
        print(f"phase93 pre-raw failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
