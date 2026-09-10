#!/usr/bin/env python3
"""One-shot Phase143 structural raw executor.

The authorization in this module is the only boundary at which route payloads
may be opened.  Before the authorization commit is verified this program can
inspect sealed JSON/source artifacts only.  After that boundary it hashes each
authorized raw input once, launches the pinned native binary once per route,
and records only structural/native metadata plus an opaque solution seal.
Truth, MAT, PDC input, precomputed coordinates, accuracy, network, retry,
fallback, and solution-coordinate parsing are deliberately absent.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import re
import subprocess
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
AUTH = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_raw_authorization_v1.json"
)
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase143_optimizer_termination_raw_result_v1.json"
)
STATIC_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase143_optimizer_termination_structural.py"
)
PHASE138_EXECUTOR = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase138_affine_tdcp_structural_authorized_execute.py"
)
OPAQUE_OUTPUT_NAME = "opaque_solution_output.csv"
LEGITIMATE_PDC_ALGORITHM_OPTION = "--native-pdc-imu-tdcp-no-bridge"
RAW_INPUT_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs")
RAW_METADATA_KEYS = frozenset({
    "path", "bytes", "sha256", "source", "read_before_authorization",
    "copy_or_transform",
})
BASE_METADATA_KEYS = frozenset({
    "path", "bytes", "sha256", "interval_s", "moving_mean_samples",
    "source", "read_before_authorization", "hash_read_before_authorization",
    "copy_or_transform",
})
RAW_SOURCE = "sealed raw-only input lineage"
BASE_SOURCE = "raw base RINEX; station coordinate from header only"
FORBIDDEN_INPUT_TERMS = (
    ".mat", "truth", "ground_truth", "pdc", "precomputed", "kaggle", "token",
)
AUTH_SCHEMA = "smartphone-r5-phase143-optimizer-termination-raw-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase143-optimizer-termination-raw-result.v1"
AUTH_COMMIT = "pending-independent-authorization-commit"

PINS = {
    "design_freeze_commit": "03073a14590f0c6316e7a0294f6b575b796b7c11",
    "implementation_commit": "f07a80bb7e26fb0502454b3cb8dc291c53545714",
    "source_audit_commit": "11911df8f0c791af136c7892c46d547f28dbeee8",
    "structural_freeze_commit": "95a24271ae3aa1c8dc64785f19e3aa8a475c8beb",
    "runner_manifest_commit": "bb2c5d6f0c46e6a683ca7a7bca59be79a1a4633c",
    "pre_raw_commit": "d514d9c4e547fbf659ebfafc70b29e7510ad1f79",
    "target_binary_sha256":
        "15ab7b401f16476928643bf41485e168a4a8be28bdb6fd1b2f8f7e26e166ef20",
}


class AuthorizationError(ValueError):
    """A fail-closed authorization, input, or structural violation."""


def fail(message: str) -> AuthorizationError:
    return AuthorizationError(message)


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load {name}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


STATIC = load_module(STATIC_PATH, "phase143_static_contract")
PHASE138 = load_module(PHASE138_EXECUTOR, "phase138_executor_helpers")


def reject_duplicate_pairs(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise fail(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def read_object(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8"), object_pairs_hook=reject_duplicate_pairs
        )
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, AuthorizationError) as exc:
        raise fail(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def digest_static(path: Path, label: str) -> str:
    """Hash a pinned static artifact; never use this for route payloads here."""
    if path.name.lower().endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"payload hash forbidden for {label}")
    if any(term in str(path).lower() for term in ("truth", "ground_truth", "precomputed")):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing static artifact {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_payload(path: Path, label: str) -> tuple[str, int]:
    """Open exactly one authorized payload once and return only hash and size."""
    if not path.is_file():
        raise fail(f"{label}: missing authorized input {path}")
    digest = hashlib.sha256()
    size = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            digest.update(chunk)
    return digest.hexdigest(), size


def digest_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def opaque_solution_metadata(path: Path) -> dict[str, Any] | None:
    """Hash/count lines only; no solution field or coordinate is interpreted."""
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    size = 0
    newlines = 0
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            size += len(chunk)
            newlines += chunk.count(b"\n")
            digest.update(chunk)
    return {
        "path": str(path.relative_to(ROOT)),
        "sha256": digest.hexdigest(),
        "bytes": size,
        "newline_count": newlines,
        "content_interpreted": False,
    }


def required(mapping: Mapping[str, Any], key: str, label: str) -> Any:
    if key not in mapping:
        raise fail(f"{label}/{key}: missing")
    return mapping[key]


def mapping(mapping_value: Mapping[str, Any], key: str, label: str) -> Mapping[str, Any]:
    value = required(mapping_value, key, label)
    if not isinstance(value, Mapping):
        raise fail(f"{label}/{key}: expected object")
    return value


def exact_keys(value: Mapping[str, Any], expected: frozenset[str], label: str) -> None:
    if set(value) != set(expected):
        raise fail(f"{label}: expected keys {sorted(expected)}, got {sorted(value)}")


def string(mapping_value: Mapping[str, Any], key: str, label: str) -> str:
    value = required(mapping_value, key, label)
    if not isinstance(value, str) or not value:
        raise fail(f"{label}/{key}: expected nonempty string")
    return value


def positive_int(mapping_value: Mapping[str, Any], key: str, label: str) -> int:
    value = required(mapping_value, key, label)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise fail(f"{label}/{key}: expected positive integer")
    return value


def sha256_value(mapping_value: Mapping[str, Any], key: str, label: str) -> str:
    value = string(mapping_value, key, label)
    if not re.fullmatch(r"[0-9a-f]{64}", value):
        raise fail(f"{label}/{key}: expected lowercase SHA256")
    return value


def repo_relative_path(mapping_value: Mapping[str, Any], key: str, label: str,
                       basename: str) -> Path:
    value = string(mapping_value, key, label)
    if value.startswith(("/", "\\")) or re.match(r"^[A-Za-z]:", value):
        raise fail(f"{label}/{key}: absolute path forbidden")
    if "\\" in value or any(part in {".", ".."} for part in value.split("/")):
        raise fail(f"{label}/{key}: traversal/separator forbidden")
    path = ROOT / Path(value)
    try:
        path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"{label}/{key}: path escapes repository") from exc
    if path.name != basename:
        raise fail(f"{label}/{key}: basename mismatch")
    if any(term in str(path).lower() for term in FORBIDDEN_INPUT_TERMS):
        raise fail(f"{label}/{key}: forbidden input category")
    return path


def validate_raw_metadata(name: str, value: Any) -> Path:
    label = f"raw/{name}"
    if not isinstance(value, Mapping):
        raise fail(f"{label}: expected flat object")
    exact_keys(value, RAW_METADATA_KEYS, label)
    path = repo_relative_path(value, "path", label, name)
    positive_int(value, "bytes", label)
    sha256_value(value, "sha256", label)
    if string(value, "source", label) != RAW_SOURCE:
        raise fail(f"{label}/source: unexpected provenance")
    if value.get("read_before_authorization") is not False:
        raise fail(f"{label}/read_before_authorization: expected false")
    if value.get("copy_or_transform") is not False:
        raise fail(f"{label}/copy_or_transform: expected false")
    return path


def validate_base_metadata(value: Any) -> Path:
    label = "base"
    if not isinstance(value, Mapping):
        raise fail(f"{label}: expected flat object")
    exact_keys(value, BASE_METADATA_KEYS, label)
    path = repo_relative_path(value, "path", label, "base.obs")
    positive_int(value, "bytes", label)
    sha256_value(value, "sha256", label)
    interval = required(value, "interval_s", label)
    if isinstance(interval, bool) or not isinstance(interval, (int, float)) or interval <= 0:
        raise fail(f"{label}/interval_s: expected positive number")
    positive_int(value, "moving_mean_samples", label)
    if string(value, "source", label) != BASE_SOURCE:
        raise fail(f"{label}/source: unexpected provenance")
    if value.get("read_before_authorization") is not False:
        raise fail(f"{label}/read_before_authorization: expected false")
    if value.get("hash_read_before_authorization") is not False:
        raise fail(f"{label}/hash_read_before_authorization: expected false")
    if value.get("copy_or_transform") is not False:
        raise fail(f"{label}/copy_or_transform: expected false")
    return path


def validate_route_record(value: Any) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    if not isinstance(value, Mapping):
        raise fail("route: expected object")
    expected_keys = {"target", "dataset_id", "runs", "raw_inputs", "base_input",
                     "output_directory"}
    exact_keys(value, frozenset(expected_keys), "route")
    target = string(value, "target", "route")
    dataset_id = string(value, "dataset_id", "route")
    expected = {"MTV-A": STATIC.ROUTES[0], "LAX-T": STATIC.ROUTES[1]}.get(target)
    if expected != dataset_id:
        raise fail("route target/dataset_id mismatch")
    if value.get("runs") != 1:
        raise fail("route/runs: expected exactly one")
    output = string(value, "output_directory", "route")
    expected_output = (
        "output/smartphone-r5/phase143-optimizer-termination-structural-v1/"
        f"{dataset_id.replace('/', '__')}"
    )
    if output != expected_output:
        raise fail("route/output_directory mismatch")
    raw = mapping(value, "raw_inputs", "route")
    base = mapping(value, "base_input", "route")
    exact_keys(raw, frozenset({"device_gnss.csv", "device_imu.csv", "brdc.nav"}),
               "route/raw_inputs")
    return raw, base


def validate_recipe(auth: Mapping[str, Any]) -> None:
    recipe = mapping(auth, "recipe", "authorization")
    on = {
        "phase135_official_affine_measurement_family",
        "phase138_affine_tdcp_anchor_range_constant",
        "phase118_official_tdcp_huber_k",
        "phase143_official_main_lm_termination_budget",
        "phase107_raw_base_compensation",
        "phase107_raw_base_source_miss_mask",
    }
    off = {
        "phase117_dynamic_tdcp_sigma", "phase120_official_tdcp_resl_atmosphere_cancellation",
        "phase126_raw_base_source_complete", "phase127_glonass_channel_provenance",
        "phase128_glonass_provenance_parser_admission", "phase129_glonass_local_miss_mask",
        "phase130_shared_ledger_key_local_support", "phase131_canonical_correction_band_key",
        "phase132_typed_canonical_preflight", "phase133_runner_native_selector_boundary",
        "phase134_native_summary_bridge", "phase107_preserve_additional_frequency_bands",
    }
    for key in on:
        if recipe.get(key) is not True:
            raise fail(f"recipe/{key}: required true")
    for key in off:
        if recipe.get(key) is not False:
            raise fail(f"recipe/{key}: required false")
    for key, expected in {
        "phase118_fixed_tdcp_sigma_m": 0.03,
        "phase118_highway_huber_k": 0.5,
        "main_linear_solver": "MULTIFRONTAL_QR",
        "main_elimination": "EliminateQR",
        "phase143_main_effective_max_iterations": 1000,
        "phase143_gnss_first_effective_max_iterations": 1000,
        "timeout_policy": "no artificial timeout shorter than native contract",
        "c7_dimension": 7,
        "c_units": "metres",
        "d_units": "metres/second",
        "pixel5_offset_applications": 1,
    }.items():
        if recipe.get(key) != expected:
            raise fail(f"recipe/{key}: expected {expected!r}")


def verify_authorization(auth: Mapping[str, Any], verify_payloads: bool = False) -> None:
    if auth.get("schema_version") != AUTH_SCHEMA:
        raise fail("authorization schema mismatch")
    if auth.get("phase") != 143:
        raise fail("authorization phase mismatch")
    if auth.get("status") != "independent-one-shot-structural-raw-authorized":
        raise fail("authorization status mismatch")
    if auth.get("route_order") != ["MTV-A", "LAX-T"] or auth.get("runs_per_route") != 1:
        raise fail("authorization route order/runs mismatch")
    routes = required(auth, "routes", "authorization")
    if not isinstance(routes, list) or len(routes) != 2:
        raise fail("authorization must contain exactly two routes")
    for index, target in enumerate(("MTV-A", "LAX-T")):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != target:
            raise fail(f"authorization/routes/{index}: order mismatch")
        raw, base = validate_route_record(record)
        for name, value in raw.items():
            validate_raw_metadata(name, value)
        validate_base_metadata(base)

    authorization = mapping(auth, "authorization", "authorization")
    for key in ("implementation", "contract", "raw_materialization",
                "raw_structural_execution", "solver"):
        if authorization.get(key) is not True:
            raise fail(f"authorization/{key}: required true")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        if authorization.get(key) is not False:
            raise fail(f"authorization/{key}: required false")

    scope = mapping(auth, "authorization_scope", "authorization")
    for key, expected in {
        "candidate_id": "phase143-official-main-lm-termination-budget-v1",
        "route_order": ["MTV-A", "LAX-T"], "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0, "repairs": 0, "sweeps": 0,
    }.items():
        if scope.get(key) != expected:
            raise fail(f"authorization_scope/{key}: expected {expected!r}")

    pins = mapping(auth, "pins", "authorization")
    for key, expected in PINS.items():
        if pins.get(key) != expected:
            raise fail(f"pins/{key}: expected {expected}")
    static_paths = {
        "source_audit_sha256": STATIC.AUDIT,
        "structural_freeze_sha256": STATIC.FREEZE,
        "design_freeze_sha256": STATIC.DESIGN_FREEZE,
        "runner_sha256": STATIC.RUNNER,
        "focused_tests_sha256": STATIC.FOCUSED_TESTS,
        "manifest_sha256": STATIC.MANIFEST,
        "pre_raw_sha256": STATIC.PRE_RAW,
        "target_binary_sha256": STATIC.BINARY,
    }
    for key, path in static_paths.items():
        expected = pins.get(key)
        if not isinstance(expected, str) or digest_static(path, key) != expected:
            raise fail(f"pins/{key}: static digest mismatch")
    executor_sha = pins.get("authorized_executor_sha256")
    if not isinstance(executor_sha, str) or digest_static(Path(__file__), "authorized executor") != executor_sha:
        raise fail("pins/authorized_executor_sha256: executor digest mismatch")

    # This reads only source and sealed launch-free metadata.  It must report
    # zero route payload/solver/truth reads before this function returns.
    launch_free = STATIC.launch_free_validation()
    if launch_free.get("raw_execution_authorized") is not False:
        raise fail("structural contract is already raw-authorized")
    if any(launch_free.get(key) != 0 for key in (
            "solver_invocations", "raw_phone_gnss_reads", "raw_phone_imu_reads",
            "broadcast_navigation_reads", "raw_base_rinex_reads", "truth_reads",
            "solution_coordinate_reads", "mat_pdc_precomputed_coordinate_reads",
            "accuracy_calculations", "kaggle_access")):
        raise fail("launch-free accounting is nonzero")
    validate_recipe(auth)
    forbidden = mapping(auth, "forbidden", "authorization")
    for key in ("truth", "MAT", "PDC", "precomputed_coordinates", "accuracy",
                "Kaggle", "solution_publication"):
        if forbidden.get(key) is not True:
            raise fail(f"forbidden/{key}: required true")
    accounting = mapping(auth, "pre_authorization_read_accounting", "authorization")
    for key, value in accounting.items():
        if isinstance(value, bool) and value is not False:
            raise fail(f"pre_authorization/{key}: nonzero")
        if isinstance(value, int) and value != 0:
            raise fail(f"pre_authorization/{key}: nonzero")
        if not isinstance(value, (bool, int)) and value not in {
                "sealed-metadata-only", "read-only"}:
            raise fail(f"pre_authorization/{key}: invalid marker")

    if verify_payloads:
        # This is the explicit post-authorization opening boundary.  No caller
        # invokes it during --verify-authorization.
        for record in routes:
            if not isinstance(record, Mapping):
                raise fail("route record disappeared after authorization")
            raw, base = validate_route_record(record)
            for name in ("device_gnss.csv", "device_imu.csv", "brdc.nav"):
                path = validate_raw_metadata(name, raw[name])
                actual, size = digest_payload(path, name)
                expected = raw[name]
                if actual != expected["sha256"] or size != expected["bytes"]:
                    raise fail(f"{name}: sealed hash/size mismatch")
            path = validate_base_metadata(base)
            actual, size = digest_payload(path, "base.obs")
            if actual != base["sha256"] or size != base["bytes"]:
                raise fail("base.obs: sealed hash/size mismatch")


def command_for(route: str, paths: Mapping[str, Path], base_sha: str,
                output_dir: Path) -> list[str]:
    command = list(STATIC.command_template(route))
    replacements = {
        STATIC.RAW_PLACEHOLDERS["--android-gnss"]: str(paths["device_gnss.csv"]),
        STATIC.RAW_PLACEHOLDERS["--android-imu"]: str(paths["device_imu.csv"]),
        STATIC.RAW_PLACEHOLDERS["--nav"]: str(paths["brdc.nav"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex"]: str(paths["base.obs"]),
        STATIC.RAW_PLACEHOLDERS["--native-base-rinex-sha256"]: base_sha,
    }
    command = [replacements.get(item, item) for item in command]
    command[command.index("--out") + 1] = str(output_dir / OPAQUE_OUTPUT_NAME)
    command[command.index("--summary-json") + 1] = str(output_dir / "native_summary.json")
    return command


def validate_command(command: list[str], route: str) -> None:
    frozen = STATIC.command_template(route)
    if len(command) != len(frozen):
        raise fail(f"{route}: argv length changed")
    for index, (actual, expected) in enumerate(zip(command, frozen)):
        if expected.startswith("--"):
            if actual != expected:
                raise fail(f"{route}: frozen option changed at {index}")
            if "pdc" in actual.lower() and actual != LEGITIMATE_PDC_ALGORITHM_OPTION:
                raise fail(f"{route}: forbidden PDC option {actual}")
        elif actual.startswith("--"):
            raise fail(f"{route}: unknown option injected at {index}")
    for selector in STATIC.REQUIRED_RECIPE_FLAGS + STATIC.ON_SELECTORS:
        if command.count(selector) != 1:
            raise fail(f"{route}: selector {selector} count != 1")
    for selector in STATIC.OFF_SELECTORS:
        if command.count(selector) != 0:
            raise fail(f"{route}: selector {selector} unexpectedly active")
    if any(_forbidden_value(value) for value in command
           if not value.startswith("--") and Path(value).name != OPAQUE_OUTPUT_NAME):
        raise fail(f"{route}: forbidden argv value")


def _forbidden_value(value: str) -> bool:
    return any(term in value.lower() for term in FORBIDDEN_INPUT_TERMS)


def normalize_summary(route: str, native: Mapping[str, Any], return_code: int,
                      opaque: Mapping[str, Any] | None,
                      reads: Mapping[str, int]) -> dict[str, Any]:
    normalized = PHASE138.normalize_native_summary(
        route, native, return_code, opaque, reads
    )
    termination = required(native, "phase143_termination", "native")
    if not isinstance(termination, Mapping):
        raise fail("native/phase143_termination: expected object")
    # Keep the authoritative termination object untouched.  The retained
    # Phase138 projection is still checked for all existing structural gates.
    normalized["phase143_termination"] = termination
    for stage_name, target in (("gnss_first", "gnss_first"), ("main", "main")):
        report = termination.get(stage_name)
        if not isinstance(report, Mapping):
            raise fail(f"native/phase143_termination/{stage_name}: expected object")
        projected = normalized["solver"][target]
        projected["accepted_iterations"] = report.get("accepted_outer_iterations")
        projected["initial_cost"] = report.get("initial_cost")
        projected["final_cost"] = report.get("final_cost")
        projected["no_fallback"] = report.get("no_fallback") is True
    STATIC.validate_structural_summary(route, normalized)
    return normalized


def run_route(route_record: Mapping[str, Any]) -> dict[str, Any]:
    route = str(required(route_record, "dataset_id", "route"))
    raw, base = validate_route_record(route_record)
    paths = {
        name: validate_raw_metadata(name, raw[name])
        for name in ("device_gnss.csv", "device_imu.csv", "brdc.nav")
    }
    paths["base.obs"] = validate_base_metadata(base)
    reads = {name: 0 for name in RAW_INPUT_NAMES}
    hashes: dict[str, Any] = {}
    for name in RAW_INPUT_NAMES:
        actual, size = digest_payload(paths[name], name)
        reads[name] = 1
        expected = base if name == "base.obs" else raw[name]
        hashes[name] = {
            "sha256": actual,
            "bytes": size,
            "expected_sha256": expected["sha256"],
            "expected_bytes": expected["bytes"],
            "match": actual == expected["sha256"] and size == expected["bytes"],
        }
        if not hashes[name]["match"]:
            return {
                "route": route, "status": "fail-closed-input-hash",
                "solver_invocations": 0, "input_read_counts": reads,
                "input_hashes": hashes,
                "failure": f"{name}: sealed hash/size mismatch",
            }

    output_dir = ROOT / (
        "output/smartphone-r5/phase143-optimizer-termination-structural-v1/"
        f"{route.replace('/', '__')}"
    )
    output_paths = (output_dir / OPAQUE_OUTPUT_NAME,
                    output_dir / "native_summary.json",
                    output_dir / "native.stdout.log",
                    output_dir / "native.stderr.log")
    if any(path.exists() for path in output_paths):
        return {
            "route": route, "status": "fail-closed-existing-output",
            "solver_invocations": 0, "input_read_counts": reads,
            "input_hashes": hashes, "failure": "one-shot output exists",
        }
    try:
        output_dir.mkdir(parents=True, exist_ok=False)
        command = command_for(route, paths, hashes["base.obs"]["sha256"], output_dir)
        validate_command(command, route)
    except (OSError, AuthorizationError) as exc:
        return {
            "route": route, "status": "fail-closed-command-preflight",
            "solver_invocations": 0, "input_read_counts": reads,
            "input_hashes": hashes, "failure": str(exc),
        }

    environment = os.environ.copy()
    local_library = "/home/sasaki/.local/lib"
    environment["LD_LIBRARY_PATH"] = local_library + (
        ":" + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else ""
    )
    completed = subprocess.run(command, cwd=ROOT, env=environment,
                               capture_output=True, check=False)
    stdout_path = output_dir / "native.stdout.log"
    stderr_path = output_dir / "native.stderr.log"
    stdout_path.write_bytes(completed.stdout)
    stderr_path.write_bytes(completed.stderr)
    summary_path = output_dir / "native_summary.json"
    output_path = output_dir / OPAQUE_OUTPUT_NAME
    route_result: dict[str, Any] = {
        "route": route,
        "status": "native-returned",
        "solver_invocations": 1,
        "input_read_counts": reads,
        "input_hashes": hashes,
        "execution": {
            "argv": command,
            "invocation_count": 1,
            "return_code": completed.returncode,
            "stdout_sha256": digest_bytes(completed.stdout),
            "stdout_bytes": len(completed.stdout),
            "stderr_sha256": digest_bytes(completed.stderr),
            "stderr_bytes": len(completed.stderr),
        },
        "solution_content_read": False,
        "opaque_solution": opaque_solution_metadata(output_path),
    }
    if not summary_path.is_file():
        route_result.update({
            "status": "fail-closed-missing-native-summary",
            "structural_gate": "native summary missing",
        })
        return route_result
    try:
        summary_bytes = summary_path.read_bytes()
        native = json.loads(summary_bytes, object_pairs_hook=reject_duplicate_pairs)
        if not isinstance(native, Mapping):
            raise fail("native summary is not an object")
        route_result["native_summary"] = {
            "path": str(summary_path.relative_to(ROOT)),
            "sha256": digest_bytes(summary_bytes),
            "bytes": len(summary_bytes),
            "coordinate_fields_interpreted": False,
        }
        read_accounting = {
            "raw_phone_gnss_reads": reads["device_gnss.csv"],
            "raw_phone_imu_reads": reads["device_imu.csv"],
            "broadcast_navigation_reads": reads["brdc.nav"],
            "raw_base_rinex_reads": reads["base.obs"],
            "truth_reads": 0, "solution_coordinate_reads": 0,
            "mat_pdc_precomputed_coordinate_reads": 0, "solver_invocations": 1,
            "accuracy_calculations": 0, "kaggle_access": 0,
        }
        normalized = normalize_summary(route, native, completed.returncode,
                                       route_result["opaque_solution"], read_accounting)
        route_result["structural_summary"] = normalized
        route_result["structural_gate"] = "pass"
    except (OSError, UnicodeDecodeError, json.JSONDecodeError, AuthorizationError,
            KeyError, TypeError, ValueError) as exc:
        route_result.update({"status": "fail-closed-structural-gate",
                             "structural_gate": "fail", "failure": str(exc)})
    return route_result


def execute(auth: Mapping[str, Any]) -> dict[str, Any]:
    verify_authorization(auth)
    routes = required(auth, "routes", "authorization")
    results: list[dict[str, Any]] = []
    for index, target in enumerate(("MTV-A", "LAX-T")):
        record = routes[index]
        if not isinstance(record, Mapping) or record.get("target") != target:
            raise fail(f"route order mismatch at {index}")
        # Each call is intentionally made once; a failed route is sealed and
        # never retried, while the next independent route still gets its one run.
        results.append(run_route(record))
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 143,
        "execution_label": "Luna Max",
        "status": "sealed-structural-raw-result-no-accuracy",
        "authorization_path": str(AUTH.relative_to(ROOT)),
        "authorization_sha256": digest_bytes(AUTH.read_bytes()),
        "candidate_id": "phase143-official-main-lm-termination-budget-v1",
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "routes": results,
        "policy": {
            "truth_used": False, "mat_used": False, "pdc_used": False,
            "precomputed_coordinates_used": False, "accuracy_evaluation": False,
            "kaggle_access": False, "solution_publication": False,
            "rerun": False, "fallback": False, "repair": False,
            "solution_coordinate_interpretation": False,
        },
        "read_accounting": {
            "raw_phone_gnss_reads": sum(r.get("input_read_counts", {}).get("device_gnss.csv", 0) for r in results),
            "raw_phone_imu_reads": sum(r.get("input_read_counts", {}).get("device_imu.csv", 0) for r in results),
            "broadcast_navigation_reads": sum(r.get("input_read_counts", {}).get("brdc.nav", 0) for r in results),
            "raw_base_rinex_reads": sum(r.get("input_read_counts", {}).get("base.obs", 0) for r in results),
            "native_solver_invocations": sum(r.get("solver_invocations", 0) for r in results),
            "truth_reads": 0, "solution_coordinate_reads": 0,
            "accuracy_calculations": 0, "mat_pdc_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0, "reruns_fallbacks_repairs_sweeps": 0,
        },
        "solution_policy": "opaque hash/bytes/newline seal only; no coordinate fields read or published",
        "authorization_commit_is_separate": True,
        "rerun_or_fallback": False,
    }
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n",
                      encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify pins/contract only; never opens route payloads")
    args = parser.parse_args(argv)
    try:
        auth = read_object(AUTH, "authorization")
        verify_authorization(auth)
        if args.verify_authorization:
            print(json.dumps({
                "status": "authorization-verified-no-payload-read",
                "raw_payload_reads": 0, "solver_invocations": 0,
                "truth_reads": 0, "solution_coordinate_reads": 0,
            }, sort_keys=True))
            return 0
        result = execute(auth)
        print(json.dumps({
            "status": result["status"],
            "route_statuses": [item.get("status") for item in result["routes"]],
            "read_accounting": result["read_accounting"],
        }, sort_keys=True))
        return 0
    except AuthorizationError as exc:
        print(f"PHASE143_RAW_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
