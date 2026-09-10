#!/usr/bin/env python3
"""Launch-free Phase133 runner/native selector boundary contract.

Phase132's sealed command forwarded a selector that belonged to the Python
contract/runner but was not implemented by the native binary.  This module
owns the corrected command snapshot: it removes that one token and validates
the native ownership boundary before a future independent authorization.

Only static source/manifest pins and synthetic in-memory argv/evidence are
handled here.  This module never materializes raw inputs, opens solution or
truth rows, or launches the native process.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUDIT = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_audit_v1.md"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_manifest_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_pre_raw_accounting_v1.json"
RUNNER = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase133_runner_native_selector_boundary_execute.py"
FOCUSED_TESTS = ROOT / "tests/test_smartphone_phase133_runner_native_selector_boundary.py"
NATIVE_SOURCE = ROOT / "apps/native/gnss_fgo_imu_no_base.cpp"
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"

AUDIT_COMMIT = "52d397880baea7854919d00bb69c39900af5b48d"
AUDIT_SHA256 = "43ba702f45f446e0c2268b6a638fd3972d87e14812ce9adc9409e82b979a6bad"
FREEZE_COMMIT = "8b1f75be0aaaadaedfffc5671be706dabd1106043"
FREEZE_SHA256 = "4dbf7bff3d9dc2537fd5fd88bf930d0c88a832bdd7f655dc47c4f5ed6b6110f6"
PHASE132_RESULT_COMMIT = "d28b6a17c85504fb26c5f7b3ecd6982968169643"
PHASE132_NOTE_COMMIT = "e42d61f23a335562657b926e866e058f69e2c702"
TARGET_BINARY_SHA256 = "ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e"

PHASE118_SELECTOR = "--native-phase118-official-tdcp-huber-k"
PHASE126_SELECTOR = "--native-phase126-raw-base-source-complete"
PHASE127_SELECTOR = "--native-phase127-glonass-channel-provenance"
PHASE128_SELECTOR = "--native-phase128-glonass-provenance-parser-admission"
PHASE129_SELECTOR = "--native-phase129-glonass-local-miss-mask"
PHASE130_SELECTOR = "--native-phase130-shared-ledger-key-local-support"
PHASE131_SELECTOR = "--native-phase131-canonical-correction-band-key"
PHASE117_SELECTOR = "--native-phase117-tdcp-snr-type-sigma"
PHASE120_SELECTOR = "--native-phase120-official-tdcp-resl-atmosphere-cancellation"
ADDITIONAL_SELECTOR = "--native-base-pseudorange-preserve-additional-frequency-bands"

NATIVE_SELECTORS = (
    PHASE118_SELECTOR,
    PHASE126_SELECTOR,
    PHASE127_SELECTOR,
    PHASE128_SELECTOR,
    PHASE129_SELECTOR,
    PHASE131_SELECTOR,
)
RUNNER_ONLY_SELECTORS = (PHASE130_SELECTOR,)
OFF_SELECTORS = (PHASE117_SELECTOR, PHASE120_SELECTOR, ADDITIONAL_SELECTOR)
OTHER_FORBIDDEN_SELECTORS = (
    "--native-direct-wls-ephemeral-c7d-main-seed",
    "--native-pdc-state-bridge",
)

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
)
ROUTE_LABELS = {
    ROUTES[0]: "MTV-A",
    ROUTES[1]: "LAX-T",
}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav")
SCHEMA = "smartphone-r5-phase133-runner-native-selector-boundary-manifest.v1"
CANDIDATE_ID = "phase133-runner-native-selector-boundary-v1"

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

# These are the non-selector recipe flags which were present in the sealed
# Phase132 launch-free command.  They are intentionally not reconstructed by
# a native process here; they are only a deterministic argv snapshot.
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


class Phase133ContractError(ValueError):
    """A Phase133 contract violation which must fail closed."""


def fail(message: str) -> Phase133ContractError:
    return Phase133ContractError(message)


def relative(path: Path) -> str:
    return str(path.relative_to(ROOT))


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value: Any, label: str) -> None:
    if value is not True:
        raise fail(f"{label}: expected true")


def finite_number(value: Any, label: str) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        raise fail(f"{label}: expected finite number")
    result = float(value)
    if not math.isfinite(result):
        raise fail(f"{label}: expected finite number")
    return result


def sha256_static(path: Path, label: str) -> str:
    """Hash source/static artifacts only; reject payload-like names."""
    lowered = path.name.lower()
    if (
        lowered in RAW_NAMES
        or lowered in {"base.obs", "truth.csv", "ground_truth.csv"}
        or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
        or "truth" in lowered
        or "ground_truth" in lowered
    ):
        raise fail(f"payload hash forbidden in launch-free validator: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def native_help_cross_check(help_text: str) -> dict[str, Any]:
    """Check a native ``--help``-shaped text without launching a binary.

    The caller may supply a synthetic fake-binary help response in tests.  A
    full C++ source text is also accepted because it contains the usage
    literals and parser ownership evidence.  Phase130 must be absent from
    both forms of native evidence.
    """
    if not isinstance(help_text, str) or not help_text.strip():
        raise fail("native help text is empty")
    required = list(NATIVE_SELECTORS)
    missing = [flag for flag in required if flag not in help_text]
    if missing:
        raise fail(f"native help/parser missing selectors: {missing}")
    if PHASE130_SELECTOR in help_text:
        raise fail("runner-only Phase130 selector appears in native help/parser")
    return {
        "required_native_selectors": required,
        "phase130_present": False,
        "real_binary_launched": False,
    }


def native_source_cross_check() -> dict[str, Any]:
    """Read and hash only the native source ownership evidence."""
    try:
        source_text = NATIVE_SOURCE.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError) as exc:
        raise fail(f"failed to read native source: {exc}") from exc
    result = native_help_cross_check(source_text)
    digest = hashlib.sha256(source_text.encode("utf-8")).hexdigest()
    assert_equal(digest, "6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170",
                 "native source sha256")
    result.update({
        "source_path": relative(NATIVE_SOURCE),
        "source_sha256": digest,
        "usage_parser_static_only": True,
    })
    return result


def selector_ownership() -> dict[str, Any]:
    return {
        "runner_only": list(RUNNER_ONLY_SELECTORS),
        "native_exactly_once": list(NATIVE_SELECTORS),
        "off": list(OFF_SELECTORS),
        "other_forbidden": list(OTHER_FORBIDDEN_SELECTORS),
    }


def command_snapshot() -> dict[str, Any]:
    return {
        "phase132": ["phase118", "phase126", "phase127", "phase128",
                     "phase129", "phase130", "phase131"],
        "phase133": ["phase118", "phase126", "phase127", "phase128",
                     "phase129", "phase131"],
        "removed": PHASE130_SELECTOR,
    }


def command_template(route: str) -> list[str]:
    """Build the fixed argv snapshot for a future independent authorization."""
    if route not in ROUTES:
        raise fail(f"unknown route: {route}")
    output_route = route.replace("/", "__")
    output_root = "output/smartphone-r5/phase133-runner-native-selector-boundary-v1/"
    # This sequence intentionally matches Phase132 except for Phase130's
    # runner-only token.  No native invocation occurs in this function.
    return [
        "build/apps/gnss_fgo_imu_no_base", "--dataset-id", route,
        "--android-gnss", "__PHASE132_RAW_DEVICE_GNSS__",
        "--android-imu", "__PHASE132_RAW_DEVICE_IMU__",
        "--nav", "__PHASE132_RAW_BROADCAST_NAV__",
        "--all-epochs", "--android-raw-utc-keys", "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback", "--native-pdc-imu-tdcp-no-bridge",
        "--native-source-direct-observable-quality", "--native-source-clock-c0d-factor",
        "--native-source-clock-c0d-meter-state-parity",
        "--native-source-clock-c0d-active-solve-diagnostic",
        "--native-source-clock-c0d-gnss-first-meter-state-handoff",
        "--native-source-clock-c0d-epoch-vector-parity",
        "--native-source-clock-c0d-phase99-main-multifrontal-qr-solver",
        "--native-base-pseudorange-compensation", "--native-base-pseudorange-source-miss-mask",
        "--native-upstream-position-offset", PHASE118_SELECTOR, PHASE126_SELECTOR,
        PHASE127_SELECTOR, PHASE128_SELECTOR, PHASE129_SELECTOR,
        PHASE131_SELECTOR, "--native-base-rinex", "__PHASE132_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256", "__PHASE132_RAW_BASE_SHA256__",
        "--out", f"{output_root}{output_route}/opaque_solution_output.csv",
        "--summary-json", f"{output_root}{output_route}/structural_summary.json",
    ]


def _option_tokens(command: Sequence[str]) -> Iterable[str]:
    for token in command:
        if isinstance(token, str) and token.startswith("--"):
            yield token


def fake_binary_unknown_options(command: Sequence[str],
                                supported_options: Iterable[str]) -> list[str]:
    """Return options a synthetic fake binary would reject.

    This models only argv parsing.  It never executes a child process and is
    deliberately used by focused tests to catch accidental Phase130
    forwarding.
    """
    supported = set(supported_options)
    return [token for token in _option_tokens(command) if token not in supported]


def fake_native_supported_options() -> set[str]:
    """The source-locked option set used by the synthetic fake binary."""
    return {
        "--dataset-id", "--android-gnss", "--android-imu", "--nav",
        "--out", "--summary-json", "--native-base-rinex",
        "--native-base-rinex-sha256", *REQUIRED_RECIPE_FLAGS, *NATIVE_SELECTORS,
    }


def validate_command(route: str, command: Any) -> None:
    if not isinstance(command, list) or any(not isinstance(item, str) for item in command):
        raise fail(f"command/{route} is not a string argv list")
    assert_equal(command, command_template(route), f"command/{route}")
    for token in command:
        if token in RUNNER_ONLY_SELECTORS or token in OFF_SELECTORS or token in OTHER_FORBIDDEN_SELECTORS:
            raise fail(f"forbidden or runner-only selector forwarded: {route}/{token}")
        if not token.startswith("--") and any(term in token.lower() for term in FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden path token: {route}/{token}")
    for flag in REQUIRED_RECIPE_FLAGS + NATIVE_SELECTORS:
        assert_equal(command.count(flag), 1, f"command/{route}/{flag}")
    for flag in RUNNER_ONLY_SELECTORS + OFF_SELECTORS:
        assert_equal(command.count(flag), 0, f"command/{route}/{flag}")
    for flag, placeholder in {
        "--android-gnss": "__PHASE132_RAW_DEVICE_GNSS__",
        "--android-imu": "__PHASE132_RAW_DEVICE_IMU__",
        "--nav": "__PHASE132_RAW_BROADCAST_NAV__",
        "--native-base-rinex": "__PHASE132_RAW_BASE_RINEX__",
        "--native-base-rinex-sha256": "__PHASE132_RAW_BASE_SHA256__",
    }.items():
        assert_equal(command[command.index(flag) + 1], placeholder,
                     f"command/{route}/{flag}/placeholder")


def verify_freeze() -> dict[str, Any]:
    assert_equal(sha256_static(AUDIT, "Phase133 audit"), AUDIT_SHA256,
                 "freeze/audit_sha256")
    freeze = read_json(FREEZE, "Phase133 freeze")
    for key, expected in {
        "schema_version": "smartphone-r5-phase133-runner-native-selector-boundary-freeze.v1",
        "phase": 133,
        "execution_label": "Luna Max",
        "status": "sealed-launch-free-boundary-freeze",
    }.items():
        assert_equal(freeze.get(key), expected, f"freeze/{key}")
    audit = freeze.get("audit")
    if not isinstance(audit, dict):
        raise fail("freeze/audit missing")
    for key, expected in {
        "path": relative(AUDIT), "commit": AUDIT_COMMIT, "sha256": AUDIT_SHA256,
    }.items():
        assert_equal(audit.get(key), expected, f"freeze/audit/{key}")
    starting = freeze.get("starting_state")
    if not isinstance(starting, dict):
        raise fail("freeze/starting_state missing")
    for key, expected in {
        "phase132_result_commit": PHASE132_RESULT_COMMIT,
        "phase132_note_commit": PHASE132_NOTE_COMMIT,
        "phase132_forensic_freeze_commit": "3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06",
        "phase132_implementation_commit": "4f546734771ac56d8342aadb88d6c36f454540fa",
        "worktree_before_freeze": "clean", "raw_reads_before_freeze": 0,
        "solver_invocations_before_freeze": 0,
    }.items():
        assert_equal(starting.get(key), expected, f"freeze/starting_state/{key}")
    decision = freeze.get("decision")
    if not isinstance(decision, dict):
        raise fail("freeze/decision missing")
    for key, expected in {
        "candidate_count": 1, "candidate_id": CANDIDATE_ID,
        "candidate_class": "runner-only argv ownership correction",
        "source_backed": True, "implementation_authorized": True,
        "raw_materialization_authorized": False, "solver_execution_authorized": False,
        "truth_evaluation_authorized": False, "accuracy_authorized": False,
        "solution_publication_authorized": False, "kaggle_authorized": False,
        "default_off": True, "partial_selector_set_allowed": False,
        "fallback_or_rerun_allowed": False,
    }.items():
        assert_equal(decision.get(key), expected, f"freeze/decision/{key}")
    ownership = freeze.get("selector_ownership")
    if not isinstance(ownership, dict):
        raise fail("freeze/selector_ownership missing")
    assert_equal(ownership.get("runner_only"), list(RUNNER_ONLY_SELECTORS),
                 "freeze/selector_ownership/runner_only")
    assert_equal(ownership.get("native_exactly_once"), list(NATIVE_SELECTORS),
                 "freeze/selector_ownership/native_exactly_once")
    assert_equal(ownership.get("forbidden_and_off"), list(OFF_SELECTORS),
                 "freeze/selector_ownership/forbidden_and_off")
    boundary = freeze.get("argv_boundary")
    if not isinstance(boundary, dict):
        raise fail("freeze/argv_boundary missing")
    assert_equal(boundary.get("removed_token"), PHASE130_SELECTOR,
                 "freeze/argv_boundary/removed_token")
    assert_equal(boundary.get("removed_token_count"), 1,
                 "freeze/argv_boundary/removed_token_count")
    for key in ("all_other_recipe_tokens_unchanged", "native_source_changed",
                "native_binary_changed"):
        expected = True if key == "all_other_recipe_tokens_unchanged" else False
        assert_equal(boundary.get(key), expected, f"freeze/argv_boundary/{key}")
    help_policy = freeze.get("native_help_cross_check")
    if not isinstance(help_policy, dict):
        raise fail("freeze/native_help_cross_check missing")
    assert_equal(help_policy.get("source_sha256"),
                 "6fcee581af70535b8af09a674a234352abaaf83a34b722d4e91577abaf352170",
                 "freeze/native_help_cross_check/source_sha256")
    assert_equal(help_policy.get("phase130_usage_present"), False,
                 "freeze/native_help_cross_check/phase130_usage_present")
    assert_equal(help_policy.get("phase130_parser_present"), False,
                 "freeze/native_help_cross_check/phase130_parser_present")
    preserved = freeze.get("preserved_native_recipe")
    if not isinstance(preserved, dict):
        raise fail("freeze/preserved_native_recipe missing")
    for key, expected in {
        "phase118_official_huber_k": True, "phase117_dynamic_sigma": False,
        "phase120_atmosphere_cancellation": False, "additional_frequency_bands": False,
        "fixed_tdcp_sigma_m": 0.03, "main_solver": "MULTIFRONTAL_QR",
        "c7_d_c0d_ccdd_unchanged": True, "raw_base_source_complete_unchanged": True,
        "phase126_127_128_129_131_118_each_once": True,
        "imu_tdcp_lm_filter_unchanged": True, "pixel5_offset_unchanged": True,
        "legacy_default_unchanged": True, "factor_topology_unchanged": True,
        "equations_units_sigma_unchanged": True,
    }.items():
        assert_equal(preserved.get(key), expected, f"freeze/preserved_native_recipe/{key}")
    routes = freeze.get("routes")
    if not isinstance(routes, list) or [item.get("target") for item in routes] != ["MTV-A", "LAX-T"]:
        raise fail("freeze route order/count changed")
    for item in routes:
        if item.get("dataset_id") not in ROUTES:
            raise fail(f"freeze unknown route: {item.get('dataset_id')}")
        assert_equal(item.get("runs"), 1, f"freeze/{item.get('target')}/runs")
        assert_equal(item.get("solver_invocations_if_preflight_fails"), 0,
                     f"freeze/{item.get('target')}/solver_invocations_if_preflight_fails")
    accounting = freeze.get("read_accounting")
    if not isinstance(accounting, dict):
        raise fail("freeze/read_accounting missing")
    for key, value in accounting.items():
        if isinstance(value, str):
            assert_equal(value, "read-only", f"freeze/read_accounting/{key}")
        elif isinstance(value, bool):
            assert_equal(value, False, f"freeze/read_accounting/{key}")
        else:
            assert_equal(value, 0, f"freeze/read_accounting/{key}")
    return freeze


def verify_static_sources() -> dict[str, str]:
    native = native_source_cross_check()
    binary_sha = sha256_static(BINARY, "target binary")
    assert_equal(binary_sha, TARGET_BINARY_SHA256, "target binary/sha256")
    return {relative(NATIVE_SOURCE): native["source_sha256"], relative(BINARY): binary_sha}


def _artifact_pin(value: Any, path: Path, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"manifest/artifacts/{label} missing")
    assert_equal(value.get("path"), relative(path), f"manifest/artifacts/{label}/path")
    digest = value.get("sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise fail(f"manifest/artifacts/{label}/sha256 missing")
    assert_equal(sha256_static(path, f"manifest/artifacts/{label}"), digest,
                 f"manifest/artifacts/{label}/sha256")


def verify_manifest() -> dict[str, Any]:
    verify_freeze()
    source_pins = verify_static_sources()
    manifest = read_json(MANIFEST, "Phase133 manifest")
    for key, expected in {
        "schema_version": SCHEMA, "phase": 133, "execution_label": "Luna Max",
        "status": "sealed-before-independent-authorization",
    }.items():
        assert_equal(manifest.get(key), expected, f"manifest/{key}")
    for section, expected in (
        ("freeze", {"path": relative(FREEZE), "commit": FREEZE_COMMIT,
                     "sha256": FREEZE_SHA256}),
        ("audit", {"path": relative(AUDIT), "commit": AUDIT_COMMIT,
                    "sha256": AUDIT_SHA256}),
    ):
        value = manifest.get(section)
        if not isinstance(value, dict):
            raise fail(f"manifest/{section} missing")
        for key, item in expected.items():
            assert_equal(value.get(key), item, f"manifest/{section}/{key}")
    implementation = manifest.get("implementation")
    if not isinstance(implementation, dict):
        raise fail("manifest/implementation missing")
    for key, expected in {
        "candidate_id": CANDIDATE_ID, "default_off": True,
        "runner_only_phase130": True, "phase130_native_argv_count": 0,
        "native_selector_argv_counts": {flag: 1 for flag in NATIVE_SELECTORS},
        "off_selector_argv_counts": {flag: 0 for flag in OFF_SELECTORS},
        "factor_topology_unchanged": True, "native_source_changed": False,
        "no_fallback": True, "no_rerun": True,
    }.items():
        assert_equal(implementation.get(key), expected, f"manifest/implementation/{key}")
    assert_equal(implementation.get("source_sha256"), source_pins,
                 "manifest/implementation/source_sha256")
    artifacts = manifest.get("artifacts")
    if not isinstance(artifacts, dict):
        raise fail("manifest/artifacts missing")
    _artifact_pin(artifacts.get("validator"), Path(__file__), "validator")
    _artifact_pin(artifacts.get("runner"), RUNNER, "runner")
    _artifact_pin(artifacts.get("focused_tests"), FOCUSED_TESTS, "focused_tests")
    ownership = manifest.get("selector_ownership")
    assert_equal(ownership, selector_ownership(), "manifest/selector_ownership")
    matrix = manifest.get("matrix")
    if not isinstance(matrix, dict):
        raise fail("manifest/matrix missing")
    for key, expected in {
        "candidate_count": 1, "route_count": 2, "runs_per_route": 1,
        "order": "MTV-A then LAX-T", "controls": 0, "reruns": 0,
        "fallbacks": 0, "sweeps": 0, "native_solver_invocations_planned": 2,
        "solution_rows_authorized": False, "truth_reads_planned": 0,
        "accuracy_calculations_planned": 0,
    }.items():
        assert_equal(matrix.get(key), expected, f"manifest/matrix/{key}")
    routes = manifest.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("manifest route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTES:
            raise fail(f"manifest unknown route: {route}")
        assert_equal(record.get("target"), ROUTE_LABELS[route], f"manifest/{route}/target")
        assert_equal(record.get("runs"), 1, f"manifest/{route}/runs")
        assert_equal(record.get("phase130_selector_count"), 0,
                     f"manifest/{route}/phase130_selector_count")
        for flag in NATIVE_SELECTORS:
            key = flag.removeprefix("--native-").replace("-", "_") + "_count"
            # The manifest also stores compact named counts; accepting the
            # selector-map above is sufficient if a future schema omits this
            # redundant field.
        validate_command(route, record.get("command"))
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"manifest/{route}/raw_inputs must be GNSS/IMU/nav only")
        for name in RAW_NAMES:
            item = raw[name]
            if not isinstance(item, dict):
                raise fail(f"manifest/{route}/{name} missing")
            assert_equal(item.get("payload_read_before_authorization"), False,
                         f"manifest/{route}/{name}/payload_read_before_authorization")
            assert_equal(item.get("copy_or_transform"), False,
                         f"manifest/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"manifest/{route}/base_input missing")
        for key, expected in {
            "placeholder": "__PHASE132_RAW_BASE_RINEX__",
            "sha256_placeholder": "__PHASE132_RAW_BASE_SHA256__",
            "raw_rinex_only": True, "payload_read_before_authorization": False,
            "hash_read_before_authorization": False, "copy_or_transform": False,
        }.items():
            assert_equal(base.get(key), expected, f"manifest/{route}/base/{key}")
    accounting = manifest.get("read_accounting_before_authorization")
    if not isinstance(accounting, dict):
        raise fail("manifest/read_accounting_before_authorization missing")
    for key, value in accounting.items():
        if isinstance(value, str):
            assert_equal(value, "read-only", f"manifest/read_accounting/{key}")
        elif isinstance(value, bool):
            assert_equal(value, False, f"manifest/read_accounting/{key}")
        else:
            assert_equal(value, 0, f"manifest/read_accounting/{key}")
    return manifest


def verify_pre_raw() -> dict[str, Any]:
    verify_manifest()
    pre_raw = read_json(PRE_RAW, "Phase133 pre-raw accounting")
    for key, expected in {
        "schema_version": "smartphone-r5-phase133-runner-native-selector-boundary-pre-raw.v1",
        "phase": 133, "status": "sealed-launch-free-zero-read",
        "authorization_commit": None,
    }.items():
        assert_equal(pre_raw.get(key), expected, f"pre_raw/{key}")
    manifest_commit = pre_raw.get("manifest_commit")
    if (not isinstance(manifest_commit, str) or len(manifest_commit) != 40
            or any(character not in "0123456789abcdef" for character in manifest_commit)):
        raise fail("pre_raw/manifest_commit must be a pinned full commit SHA")
    for key, expected in {
        "raw_phone_gnss_reads": 0, "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0, "raw_base_rinex_payload_reads": 0,
        "raw_base_header_reads": 0, "native_solver_invocations": 0,
        "solution_coordinate_row_reads": 0, "truth_reads": 0,
        "accuracy_calculations": 0, "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0, "reruns_fallbacks_repairs_sweeps": 0,
    }.items():
        assert_equal(pre_raw.get(key), expected, f"pre_raw/{key}")
    return pre_raw


def verify_execution_evidence(route: str, evidence: Mapping[str, Any],
                              *, preflight_passed: bool) -> dict[str, Any]:
    """Validate synthetic future-run call counts without launching anything."""
    if route not in ROUTES or not isinstance(evidence, Mapping):
        raise fail("execution evidence route/object invalid")
    phase = evidence.get("phase133")
    if not isinstance(phase, Mapping):
        raise fail("execution evidence/phase133 missing")
    typed = phase.get("typed_preflight_call_count")
    if not isinstance(typed, int) or isinstance(typed, bool) or typed < 1:
        raise fail("execution evidence typed preflight call count must be positive")
    assert_equal(phase.get("old_literal_phase130_preflight_call_count"), 0,
                 "execution evidence old literal Phase130 calls")
    if not preflight_passed:
        for key in ("native_command_constructed", "native_selector_forwarded",
                    "native_binary_invocation_attempted", "native_resolver_executed"):
            assert_equal(phase.get(key), False, f"execution evidence failure/{key}")
        assert_equal(phase.get("native_command_construction_count"), 0,
                     "execution evidence failure/command count")
        assert_equal(phase.get("native_solver_invocations"), 0,
                     "execution evidence failure/solver count")
        return {"route": route, "preflight_passed": False, "fail_closed": True}
    for key in ("native_command_constructed", "native_selector_forwarded",
                "native_binary_invocation_attempted"):
        assert_equal(phase.get(key), True, f"execution evidence pass/{key}")
    assert_equal(phase.get("native_command_construction_count"), 1,
                 "execution evidence pass/command count")
    assert_equal(phase.get("native_selector_forwarding_count"), 1,
                 "execution evidence pass/selector count")
    assert_equal(phase.get("native_solver_invocations"), 1,
                 "execution evidence pass/solver count")
    assert_true(phase.get("native_resolver_executed"),
                "execution evidence pass/native resolver")
    assert_true(phase.get("native_resolver_call_count", 0) > 0,
                "execution evidence pass/native resolver call count")
    return {"route": route, "preflight_passed": True, "fail_closed": False}


def zero_read_accounting() -> dict[str, Any]:
    return {
        "source_text_reads": "read-only",
        "sealed_metadata_reads": "read-only",
        "raw_phone_gnss_reads": 0,
        "raw_phone_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_payload_reads": 0,
        "raw_base_header_reads": 0,
        "raw_payload_copies_or_transforms": 0,
        "native_solver_invocations": 0,
        "solution_coordinate_row_reads": 0,
        "truth_reads": 0,
        "accuracy_calculations": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
        "kaggle_or_token_access": 0,
        "reruns_fallbacks_repairs_sweeps": 0,
    }


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--verify-manifest", action="store_true")
    parser.add_argument("--verify-pre-raw", action="store_true")
    args = parser.parse_args(argv)
    if not any((args.verify_freeze, args.verify_manifest, args.verify_pre_raw)):
        parser.error("launch-free mode requires --verify-freeze, --verify-manifest, or --verify-pre-raw")
    if args.verify_freeze:
        verify_freeze()
    if args.verify_manifest:
        verify_manifest()
    if args.verify_pre_raw:
        verify_pre_raw()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
