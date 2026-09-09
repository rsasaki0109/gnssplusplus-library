#!/usr/bin/env python3
"""Execute the separately authorized Phase147 H/U structural diagnostic.

The command is intentionally one-shot and route-independent: the H route is
allowed to fail while the U route is still attempted once.  Before the
authorization boundary this module reads only tracked source and sealed JSON
metadata.  After that boundary it reads each sealed GNSS/IMU/nav/base member
once, starts the pinned native executable once per route, and seals hashes plus
scalar native telemetry.  It never reads truth, MAT/PDC, precomputed
coordinates, or solution fields.
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
    "smartphone_r5_phase147_hu_structural_diagnostic_authorization_v1.json"
)
PLAN = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase147_hu_structural_preflight_manifest_v1.json"
)
RESULT = ROOT / (
    "docs/use_cases/records/"
    "smartphone_r5_phase147_hu_structural_diagnostic_result_v1.json"
)
PHASE143_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase143_optimizer_termination_structural.py"
)
PHASE144_PATH = ROOT / (
    "apps/commands/benchmarks/"
    "gnss_smartphone_phase144_telemetry_serializer_structural.py"
)
BINARY = ROOT / "build/apps/gnss_fgo_imu_no_base"
EXECUTOR = Path(__file__).resolve()
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase147-hu-structural-diagnostic-v1"

AUTH_SCHEMA = "smartphone-r5-phase147-hu-structural-diagnostic-authorization.v1"
RESULT_SCHEMA = "smartphone-r5-phase147-hu-structural-diagnostic-result.v1"
PHASE144_NATIVE_SCHEMA = "smartphone-r5-native-fgo-phase144-telemetry.v1"
TERMINATION_SCHEMA = "smartphone-r5-native-fgo-phase143-termination.v1"
PHASE144_SELECTOR = "--native-phase144-telemetry-schema"

ROUTES = (
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGETS = ("MTV-H", "MTV-U")
ROUTE_TYPES = {route: "Street" for route in ROUTES}
ROUTE_TDCP_K = {route: 0.2 for route in ROUTES}
RAW_NAMES = ("device_gnss.csv", "device_imu.csv", "brdc.nav", "base.obs")
RAW_MEMBER_NAMES = frozenset({"device_gnss.csv", "device_imu.csv", "brdc.nav"})
FORBIDDEN_TERMS = (
    ".mat", "truth", "ground_truth", "precomputed", "kaggle", "token",
)


class Phase147AuthorizationError(ValueError):
    """A fail-closed authorization, command, or structural violation."""


def fail(message: str) -> Phase147AuthorizationError:
    return Phase147AuthorizationError(message)


def load_module(path: Path, name: str) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise fail(f"unable to load {name}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# Reuse the frozen Phase143 argv and Phase144 native-summary validators, with
# route scope changed only in this process.  Historical A/LAX validators and
# artifacts remain untouched on disk.
PHASE143 = load_module(PHASE143_PATH, "phase143_for_phase147_hu")
PHASE144 = load_module(PHASE144_PATH, "phase144_for_phase147_hu")
PHASE143.ROUTES = ROUTES
PHASE144.ROUTES = ROUTES


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
    except (OSError, UnicodeDecodeError, json.JSONDecodeError,
            Phase147AuthorizationError) as exc:
        raise fail(f"{label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label}: expected object")
    return value


def digest_static(path: Path, label: str) -> str:
    """Hash only source/binary/contract metadata, never an input payload."""
    lowered_name = path.name.lower()
    lowered_path = str(path).lower()
    if lowered_name.endswith((".csv", ".nav", ".obs", ".mat")):
        raise fail(f"payload hash forbidden for {label}")
    if any(term in lowered_path for term in FORBIDDEN_TERMS):
        raise fail(f"forbidden artifact hash for {label}")
    if not path.is_file():
        raise fail(f"missing static artifact {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def digest_payload(path: Path, label: str) -> tuple[str, int]:
    """Hash one authorized raw member; called only after auth verification."""
    if path.name not in RAW_NAMES:
        raise fail(f"unexpected payload basename for {label}: {path.name}")
    if not path.is_file():
        raise fail(f"missing authorized input {label}: {path}")
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
    """Hash/count only; no solution CSV field or coordinate is parsed."""
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


def sha256_value(value: Any, label: str) -> str:
    if not isinstance(value, str) or not re.fullmatch(r"[0-9a-f]{64}", value):
        raise fail(f"{label}: expected lowercase SHA256")
    return value


def safe_relative_path(text: Any, basename: str, label: str) -> Path:
    if not isinstance(text, str) or not text or text.startswith("/"):
        raise fail(f"{label}: unsafe path")
    parts = text.split("/")
    if any(part in {"", ".", ".."} for part in parts):
        raise fail(f"{label}: traversal/empty path")
    path = ROOT / Path(text)
    try:
        path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"{label}: path escapes repository") from exc
    if path.name != basename:
        raise fail(f"{label}: basename mismatch")
    lowered = str(path).lower()
    if any(term in lowered for term in FORBIDDEN_TERMS):
        raise fail(f"{label}: forbidden path category")
    return path


def route_record(plan: Mapping[str, Any], route: str) -> Mapping[str, Any]:
    provenance = required(plan, "phase113_raw_provenance", "plan")
    if not isinstance(provenance, Mapping):
        raise fail("plan/phase113_raw_provenance: expected object")
    routes = required(provenance, "routes", "plan/phase113_raw_provenance")
    if not isinstance(routes, Mapping):
        raise fail("plan/phase113_raw_provenance/routes: expected object")
    record = routes.get(route)
    if not isinstance(record, Mapping):
        raise fail(f"plan/phase113_raw_provenance/routes/{route}: missing")
    return record


def route_inputs(plan: Mapping[str, Any], route: str) -> dict[str, tuple[Path, Mapping[str, Any]]]:
    record = route_record(plan, route)
    raw = required(record, "raw_inputs", f"plan/route/{route}")
    if not isinstance(raw, Mapping) or set(raw) != set(RAW_MEMBER_NAMES):
        raise fail(f"plan/route/{route}/raw_inputs: exact GNSS/IMU/nav set required")
    resolved: dict[str, tuple[Path, Mapping[str, Any]]] = {}
    for name in sorted(RAW_MEMBER_NAMES):
        item = raw[name]
        if not isinstance(item, Mapping):
            raise fail(f"plan/route/{route}/{name}: expected object")
        path = safe_relative_path(item.get("path"), name, f"plan/route/{route}/{name}")
        sha256_value(item.get("sha256"), f"plan/route/{route}/{name}/sha256")
        if (not isinstance(item.get("bytes"), int) or
                isinstance(item.get("bytes"), bool) or item["bytes"] <= 0):
            raise fail(f"plan/route/{route}/{name}/bytes: expected positive integer")
        resolved[name] = (path, item)
    base = required(record, "base_input", f"plan/route/{route}")
    if not isinstance(base, Mapping):
        raise fail(f"plan/route/{route}/base_input: expected object")
    path = safe_relative_path(base.get("path"), "base.obs", f"plan/route/{route}/base")
    sha256_value(base.get("sha256"), f"plan/route/{route}/base/sha256")
    if (not isinstance(base.get("bytes"), int) or
            isinstance(base.get("bytes"), bool) or base["bytes"] <= 0):
        raise fail(f"plan/route/{route}/base/bytes: expected positive integer")
    resolved["base.obs"] = (path, base)
    return resolved


def validate_plan(plan: Mapping[str, Any]) -> None:
    expected = {
        "schema_version": "smartphone-r5-phase147-hu-structural-preflight-manifest.v1",
        "phase": 147,
        "status": "sealed-preflight-for-future-diagnostic-only-run",
        "routes": list(ROUTES),
    }
    for key, value in expected.items():
        if plan.get(key) != value:
            raise fail(f"plan/{key}: expected {value!r}, got {plan.get(key)!r}")
    candidate = required(plan, "candidate", "plan")
    if not isinstance(candidate, Mapping):
        raise fail("plan/candidate: expected object")
    if candidate.get("planned_routes") != list(ROUTES):
        raise fail("plan/candidate/planned_routes: route order changed")
    for key in (
        "diagnostic_only", "algorithm_or_solver_change", "new_selector",
        "unknown_or_non_pixel5_rejected", "doppler_empty_state_synthesis",
        "velocity_fallback_or_synthesis", "guard_removal", "numerical_settings_changed",
        "solution_rows_published",
    ):
        if candidate.get(key) is not (True if key in {
                "diagnostic_only", "unknown_or_non_pixel5_rejected"} else False):
            raise fail(f"plan/candidate/{key}: contract changed")
    recipe = required(plan, "recipe", "plan")
    if not isinstance(recipe, Mapping):
        raise fail("plan/recipe: expected object")
    for key, value in {
        "phase107_raw_base": True,
        "phase107_source_miss_mask": True,
        "main_configured_max_iterations": 12,
        "main_effective_max_iterations": 1000,
        "gnss_first_configured_max_iterations": 1000,
        "gnss_first_effective_max_iterations": 1000,
        "main_solver": "MULTIFRONTAL_QR / EliminateQR",
    }.items():
        if recipe.get(key) != value:
            raise fail(f"plan/recipe/{key}: expected {value!r}")
    if recipe.get("official_type_mapping", {}).get("MTV-H") != "Street":
        raise fail("plan/recipe/official_type_mapping/MTV-H: Street required")
    if recipe.get("official_type_mapping", {}).get("MTV-U") != "Street":
        raise fail("plan/recipe/official_type_mapping/MTV-U: Street required")
    if recipe.get("official_type_mapping", {}).get("street_tdcp_huber_threshold_sigma") != 0.2:
        raise fail("plan/recipe/official_type_mapping: Street TDCP k changed")

    pins = required(plan, "current_source_and_binary_pins", "plan")
    if not isinstance(pins, Mapping):
        raise fail("plan/current_source_and_binary_pins: expected object")
    source_paths = (
        "apps/native/gnss_fgo_imu_no_base.cpp",
        "include/libgnss++/algorithms/fgo.hpp",
        "include/libgnss++/algorithms/fgo_config.hpp",
        "include/libgnss++/algorithms/upstream_position_offset.hpp",
        "src/algorithms/fgo_gtsam_backend.cpp",
        "src/algorithms/fgo_gtsam_internal.hpp",
        "src/algorithms/fgo_problems.cpp",
        "src/algorithms/base_pseudorange_compensation.cpp",
    )
    for path_text in source_paths:
        item = pins.get(path_text)
        if not isinstance(item, Mapping):
            raise fail(f"plan/source pin missing: {path_text}")
        expected_sha = sha256_value(item.get("sha256"), f"plan/source/{path_text}")
        actual_sha = digest_static(ROOT / path_text, f"source/{path_text}")
        if actual_sha != expected_sha:
            raise fail(f"plan/source/{path_text}: SHA256 mismatch")
    binary_pin = pins.get("build/apps/gnss_fgo_imu_no_base")
    if not isinstance(binary_pin, Mapping):
        raise fail("plan/binary pin missing")
    binary_sha = sha256_value(binary_pin.get("sha256"), "plan/binary/sha256")
    if digest_static(BINARY, "target binary") != binary_sha:
        raise fail("plan/binary: SHA256 mismatch")

    phase144 = required(plan, "phase144_reference", "plan")
    if not isinstance(phase144, Mapping):
        raise fail("plan/phase144_reference: expected object")
    manifest_ref = required(phase144, "manifest", "plan/phase144_reference")
    if not isinstance(manifest_ref, Mapping):
        raise fail("plan/phase144_reference/manifest: expected object")
    if digest_static(ROOT / str(manifest_ref.get("path")), "Phase144 manifest") != manifest_ref.get("sha256"):
        raise fail("plan/phase144_reference/manifest: SHA256 mismatch")
    provenance = required(plan, "phase113_raw_provenance", "plan")
    if not isinstance(provenance, Mapping):
        raise fail("plan/phase113_raw_provenance: expected object")
    source_manifest = required(provenance, "source_manifest", "plan/phase113_raw_provenance")
    if not isinstance(source_manifest, Mapping):
        raise fail("plan/phase113_raw_provenance/source_manifest: expected object")
    if digest_static(ROOT / str(source_manifest.get("path")), "Phase113 manifest") != source_manifest.get("sha256"):
        raise fail("plan/phase113_raw_provenance/source_manifest: SHA256 mismatch")
    for route in ROUTES:
        record = route_record(plan, route)
        if record.get("label") != {ROUTES[0]: "MTV-H", ROUTES[1]: "MTV-U"}[route]:
            raise fail(f"plan/route/{route}/label: mismatch")
        route_inputs(plan, route)

    boundary = required(plan, "execution_boundary", "plan")
    if not isinstance(boundary, Mapping):
        raise fail("plan/execution_boundary: expected object")
    for key in (
        "raw_execution_authorized", "solver_execution_authorized", "truth_authorized",
        "accuracy_authorized", "solution_publication_authorized",
        "mat_pdc_precomputed_reads_authorized", "kaggle_or_token_authorized",
    ):
        if boundary.get(key) is not False:
            raise fail(f"plan/execution_boundary/{key}: must remain false before auth")
    accounting = required(plan, "read_accounting_before_authorization", "plan")
    if not isinstance(accounting, Mapping) or any(value != 0 for value in accounting.values()):
        raise fail("plan/read_accounting_before_authorization: nonzero")


def command_for(route: str, paths: Mapping[str, Path], base_sha: str,
                output_dir: Path) -> list[str]:
    if route not in ROUTES:
        raise fail(f"unknown H/U route: {route}")
    command = list(PHASE143.command_template(route))
    command.insert(command.index(PHASE143.PHASE143) + 1, PHASE144_SELECTOR)
    replacements = {
        PHASE143.RAW_PLACEHOLDERS["--android-gnss"]: str(paths["device_gnss.csv"]),
        PHASE143.RAW_PLACEHOLDERS["--android-imu"]: str(paths["device_imu.csv"]),
        PHASE143.RAW_PLACEHOLDERS["--nav"]: str(paths["brdc.nav"]),
        PHASE143.RAW_PLACEHOLDERS["--native-base-rinex"]: str(paths["base.obs"]),
        PHASE143.RAW_PLACEHOLDERS["--native-base-rinex-sha256"]: base_sha,
    }
    command = [replacements.get(item, item) for item in command]
    command[command.index("--out") + 1] = str(output_dir / "opaque_solution_output.csv")
    command[command.index("--summary-json") + 1] = str(output_dir / "native_summary.json")
    return command


def validate_command(command: Sequence[str], route: str,
                     paths: Mapping[str, Path], base_sha: str,
                     output_dir: Path) -> None:
    expected = command_for(route, paths, base_sha, output_dir)
    if list(command) != expected:
        raise fail(f"{route}: argv differs from frozen Phase143 command plus Phase144 schema")
    for selector in PHASE143.REQUIRED_RECIPE_FLAGS + PHASE143.ON_SELECTORS + (PHASE144_SELECTOR,):
        if command.count(selector) != 1:
            raise fail(f"{route}: selector {selector} count != 1")
    for selector in PHASE143.OFF_SELECTORS:
        if command.count(selector) != 0:
            raise fail(f"{route}: forbidden selector {selector}")
    if command[command.index("--dataset-id") + 1] != route:
        raise fail(f"{route}: dataset ID mismatch")
    if command[command.index("--summary-json") + 1] != str(output_dir / "native_summary.json"):
        raise fail(f"{route}: summary path mismatch")
    for name in RAW_MEMBER_NAMES:
        flag = {"device_gnss.csv": "--android-gnss",
                "device_imu.csv": "--android-imu",
                "brdc.nav": "--nav"}[name]
        if command[command.index(flag) + 1] != str(paths[name]):
            raise fail(f"{route}: {flag} path mismatch")
    if command[command.index("--native-base-rinex") + 1] != str(paths["base.obs"]):
        raise fail(f"{route}: base path mismatch")
    if command[command.index("--native-base-rinex-sha256") + 1] != base_sha:
        raise fail(f"{route}: base SHA mismatch")
    for value in command:
        if any(term in value.lower() for term in FORBIDDEN_TERMS):
            raise fail(f"{route}: forbidden command token {value}")


def synthetic_command_for(route: str) -> list[str]:
    """Return an argv with non-materializing placeholders for preflight tests."""
    placeholders = {
        name: ROOT / f"__PHASE147_{name.replace('.', '_').upper()}__"
        for name in RAW_NAMES
    }
    output = ROOT / "__phase147_synthetic_output__"
    return command_for(route, placeholders, "0" * 64, output)


def validate_help_argv() -> dict[str, Any]:
    """Check every frozen option against the binary's own usage text."""
    try:
        environment = os.environ.copy()
        local_library = "/home/sasaki/.local/lib"
        environment["LD_LIBRARY_PATH"] = local_library + (
            ":" + environment["LD_LIBRARY_PATH"]
            if environment.get("LD_LIBRARY_PATH") else ""
        )
        completed = subprocess.run(
            [str(BINARY), "--help"], cwd=ROOT, env=environment,
            capture_output=True, check=False
        )
    except OSError as exc:
        raise fail(f"unable to run native --help preflight: {exc}") from exc
    help_text = (completed.stdout + completed.stderr).decode("utf-8", errors="replace")
    if not help_text:
        raise fail("native --help returned no usage text")
    missing: list[str] = []
    for route in ROUTES:
        for token in synthetic_command_for(route):
            if token.startswith("--") and token not in help_text:
                missing.append(token)
    if missing:
        raise fail(f"native --help missing command options: {sorted(set(missing))}")
    return {
        "binary_help_invocations": 1,
        "binary_help_return_code": completed.returncode,
        "command_options_checked": True,
        "raw_reads": 0,
        "solver_invocations": 0,
    }


def validate_authorization(auth: Mapping[str, Any]) -> Mapping[str, Any]:
    if auth.get("schema_version") != AUTH_SCHEMA:
        raise fail("authorization schema mismatch")
    if auth.get("phase") != 147 or auth.get("status") != "authorized-for-one-shot-hu-diagnostic":
        raise fail("authorization phase/status mismatch")
    if auth.get("route_order") != list(TARGETS) or auth.get("runs_per_route") != 1:
        raise fail("authorization route order/runs mismatch")
    plan_ref = required(auth, "plan", "authorization")
    if not isinstance(plan_ref, Mapping):
        raise fail("authorization/plan: expected object")
    if plan_ref.get("path") != str(PLAN.relative_to(ROOT)):
        raise fail("authorization/plan/path mismatch")
    if digest_static(PLAN, "Phase147 plan") != plan_ref.get("sha256"):
        raise fail("authorization/plan/sha256 mismatch")
    plan = read_object(PLAN, "Phase147 plan")
    validate_plan(plan)
    routes = required(auth, "routes", "authorization")
    if not isinstance(routes, list) or len(routes) != len(ROUTES):
        raise fail("authorization/routes: expected H then U exactly once")
    for index, (target, route) in enumerate(zip(TARGETS, ROUTES)):
        item = routes[index]
        if not isinstance(item, Mapping):
            raise fail(f"authorization/routes/{index}: expected object")
        for key, expected in {
            "target": target, "dataset_id": route, "runs": 1,
            "type": "Street", "tdcp_huber_threshold_sigma": 0.2,
        }.items():
            if item.get(key) != expected:
                raise fail(f"authorization/routes/{index}/{key}: expected {expected!r}")
        route_inputs(plan, route)
    recipe = required(auth, "recipe", "authorization")
    if not isinstance(recipe, Mapping):
        raise fail("authorization/recipe: expected object")
    for key, expected in {
        "phase144_selector": PHASE144_SELECTOR,
        "phase143_selector": PHASE143.PHASE143,
        "phase135_selector": PHASE143.PHASE135,
        "phase138_selector": PHASE143.PHASE138,
        "phase118_selector": PHASE143.PHASE118,
        "phase107_raw_base": True,
        "phase107_source_miss_mask": True,
        "main_configured_max_iterations": 12,
        "main_effective_max_iterations": 1000,
        "gnss_first_configured_max_iterations": 1000,
        "gnss_first_effective_max_iterations": 1000,
        "main_solver": "MULTIFRONTAL_QR / EliminateQR",
        "street_tdcp_huber_threshold_sigma": 0.2,
        "pixel5_offset_applications": 1,
    }.items():
        if recipe.get(key) != expected:
            raise fail(f"authorization/recipe/{key}: expected {expected!r}")
    authorization = required(auth, "authorization", "authorization")
    if not isinstance(authorization, Mapping):
        raise fail("authorization/authorization: expected object")
    for key in ("raw_execution", "solver_execution", "implementation", "structural_telemetry"):
        if authorization.get(key) is not True:
            raise fail(f"authorization/{key}: required true")
    for key in (
        "truth", "accuracy", "mat_pdc", "precomputed_coordinates", "kaggle_or_token",
        "solution_coordinate_interpretation", "solution_publication", "rerun", "fallback",
        "repair", "tuning", "sweep",
    ):
        if authorization.get(key) is not False:
            raise fail(f"authorization/{key}: required false")
    accounting = required(auth, "pre_authorization_read_accounting", "authorization")
    if not isinstance(accounting, Mapping) or any(value != 0 for value in accounting.values()):
        raise fail("authorization/pre_authorization_read_accounting: nonzero")
    return plan


def classify_failure(return_code: int, stderr: bytes, summary_present: bool,
                     summary_error: str | None = None) -> tuple[str, str]:
    text = stderr.decode("utf-8", errors="replace")
    if "undifferenced_doppler_factors.empty" in text or "ClockFactor_CCDD" in text:
        return "gnss-first-c0d-admission", "native C0/D undifferenced-Doppler admission failure"
    if "coverage-contract" in text or "optimized-D/progress contract" in text:
        return "main-coverage-contract", "native main coverage/progress contract failure"
    if not summary_present:
        return "native-summary-unavailable", "native summary was not produced"
    if summary_error:
        if "expected_epoch_coverage" in summary_error:
            return "output-epoch-coverage", (
                "native output expected_epoch_coverage=false"
            )
        return "native-summary-structural-contract", summary_error
    if return_code != 0:
        return "native-return-nonzero", f"native process returned {return_code}"
    return "unknown-native-failure", "native diagnostic did not satisfy a known terminal condition"


def structural_projection(native: Mapping[str, Any]) -> dict[str, Any]:
    telemetry = required(native, "phase144_telemetry", "native")
    if not isinstance(telemetry, Mapping):
        raise fail("native/phase144_telemetry: expected object")
    projection: dict[str, Any] = {}
    for key in ("phase135", "phase138", "raw_base", "clock", "solver",
                "factor_counts", "bridge", "offset", "output"):
        value = required(telemetry, key, f"native/phase144_telemetry")
        if not isinstance(value, Mapping):
            raise fail(f"native/phase144_telemetry/{key}: expected object")
        projection[key] = value
    termination = required(native, "phase143_termination", "native")
    if not isinstance(termination, Mapping):
        raise fail("native/phase143_termination: expected object")
    projection["phase143_termination"] = termination
    return projection


def run_route(plan: Mapping[str, Any], route: str, target: str) -> dict[str, Any]:
    output_dir = OUTPUT_ROOT / route.replace("/", "__")
    output_paths = (
        output_dir / "opaque_solution_output.csv",
        output_dir / "native_summary.json",
        output_dir / "native.stdout.log",
        output_dir / "native.stderr.log",
    )
    base_result: dict[str, Any] = {
        "target": target,
        "route": route,
        "type": ROUTE_TYPES[route],
        "tdcp_huber_threshold_sigma": ROUTE_TDCP_K[route],
        "status": "not-started",
        "input_read_counts": {name: 0 for name in RAW_NAMES},
        "native_process_invocations": 0,
        "summary": {"present": False, "read_count": 0},
        "opaque_solution": None,
    }
    try:
        if any(path.exists() for path in output_paths):
            raise fail("one-shot output already exists; refusing overwrite")
        inputs = route_inputs(plan, route)
        hashes: dict[str, Any] = {}
        for name in (*sorted(RAW_MEMBER_NAMES), "base.obs"):
            path, pin = inputs[name]
            actual_sha, actual_bytes = digest_payload(path, f"{route}/{name}")
            base_result["input_read_counts"][name] = 1
            hashes[name] = {
                "path": str(path.relative_to(ROOT)),
                "sha256": actual_sha,
                "bytes": actual_bytes,
                "expected_sha256": pin["sha256"],
                "expected_bytes": pin["bytes"],
                "match": actual_sha == pin["sha256"] and actual_bytes == pin["bytes"],
            }
        base_result["input_hashes"] = hashes
        mismatches = [name for name, item in hashes.items() if not item["match"]]
        if mismatches:
            base_result.update({
                "status": "fail-closed-input-hash",
                "failure_stage": "input-integrity",
                "failure_message": f"sealed hash/size mismatch: {','.join(mismatches)}",
            })
            return base_result
        output_dir.mkdir(parents=True, exist_ok=False)
        paths = {name: value[0] for name, value in inputs.items()}
        command = command_for(route, paths, hashes["base.obs"]["sha256"], output_dir)
        validate_command(command, route, paths, hashes["base.obs"]["sha256"], output_dir)
        environment = os.environ.copy()
        local_library = "/home/sasaki/.local/lib"
        environment["LD_LIBRARY_PATH"] = local_library + (
            ":" + environment["LD_LIBRARY_PATH"]
            if environment.get("LD_LIBRARY_PATH") else ""
        )
        completed = subprocess.run(
            command, cwd=ROOT, env=environment, capture_output=True, check=False
        )
        base_result["native_process_invocations"] = 1
        stdout = completed.stdout
        stderr = completed.stderr
        (output_dir / "native.stdout.log").write_bytes(stdout)
        (output_dir / "native.stderr.log").write_bytes(stderr)
        summary_path = output_dir / "native_summary.json"
        output_path = output_dir / "opaque_solution_output.csv"
        base_result["execution"] = {
            "argv": command,
            "invocation_count": 1,
            "return_code": completed.returncode,
            "stdout_sha256": digest_bytes(stdout),
            "stdout_bytes": len(stdout),
            "stderr_sha256": digest_bytes(stderr),
            "stderr_bytes": len(stderr),
        }
        base_result["opaque_solution"] = opaque_solution_metadata(output_path)
        if not summary_path.is_file():
            stage, message = classify_failure(completed.returncode, stderr, False)
            base_result.update({
                "status": "fail-closed-missing-native-summary",
                "failure_stage": stage,
                "failure_message": message,
            })
            return base_result
        summary_bytes = summary_path.read_bytes()
        base_result["summary"] = {
            "path": str(summary_path.relative_to(ROOT)),
            "sha256": digest_bytes(summary_bytes),
            "bytes": len(summary_bytes),
            "read_count": 1,
            "duplicate_paths": [],
            "coordinate_fields_interpreted": False,
            "preserved_native_file": True,
        }
        try:
            native = json.loads(
                summary_bytes.decode("utf-8"), object_pairs_hook=reject_duplicate_pairs
            )
            if not isinstance(native, Mapping):
                raise fail("native summary is not an object")
            # Preserve a scalar-only projection even when a structural gate
            # fails; the native summary itself remains byte-for-byte intact.
            base_result["structural_telemetry"] = structural_projection(native)
            PHASE144.validate_native_summary(route, native)
            if completed.returncode != 0:
                stage, message = classify_failure(completed.returncode, stderr, True)
                base_result.update({
                    "status": "fail-closed-native-return",
                    "failure_stage": stage,
                    "failure_message": message,
                })
            else:
                base_result.update({
                    "status": "structural-go",
                    "failure_stage": None,
                    "failure_message": None,
                })
        except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError,
                ValueError, Phase147AuthorizationError) as exc:
            stage, message = classify_failure(
                completed.returncode, stderr, True, str(exc)
            )
            base_result.update({
                "status": "fail-closed-structural-summary",
                "failure_stage": stage,
                "failure_message": message,
            })
    except (OSError, Phase147AuthorizationError) as exc:
        base_result.update({
            "status": "fail-closed-preflight",
            "failure_stage": "command-or-input-preflight",
            "failure_message": str(exc),
        })
    return base_result


def execute(auth: Mapping[str, Any]) -> dict[str, Any]:
    plan = validate_authorization(auth)
    results = [run_route(plan, route, target) for route, target in zip(ROUTES, TARGETS)]
    result = {
        "schema_version": RESULT_SCHEMA,
        "phase": 147,
        "execution_label": "Luna Max",
        "status": "sealed-hu-structural-diagnostic-no-truth-accuracy",
        "authorization_path": str(AUTH.relative_to(ROOT)),
        "authorization_sha256": digest_bytes(AUTH.read_bytes()),
        "plan_path": str(PLAN.relative_to(ROOT)),
        "plan_sha256": digest_static(PLAN, "Phase147 plan"),
        "candidate_id": "phase147-known-hu-route-admission-diagnostic-v1",
        "route_order": list(TARGETS),
        "runs_per_route": 1,
        "routes": results,
        "structural_policy": {
            "official_type_h_u": "Street",
            "street_tdcp_huber_threshold_sigma": 0.2,
            "pixel5_offset_reapplication": 0,
            "configured_effective_limits_preserved": True,
            "main_configured_max_iterations": 12,
            "main_effective_max_iterations": 1000,
            "gnss_first_configured_max_iterations": 1000,
            "gnss_first_effective_max_iterations": 1000,
            "accepted_iterations_authority": "native Phase143 termination reports",
            "solution_coordinate_interpretation": False,
            "truth_used": False,
            "accuracy_evaluation": False,
            "mat_pdc_precomputed_reads": False,
            "kaggle_or_token_access": False,
            "rerun": False,
            "fallback": False,
            "repair": False,
        },
        "read_accounting": {
            "raw_phone_gnss_reads": sum(r["input_read_counts"]["device_gnss.csv"] for r in results),
            "raw_phone_imu_reads": sum(r["input_read_counts"]["device_imu.csv"] for r in results),
            "broadcast_navigation_reads": sum(r["input_read_counts"]["brdc.nav"] for r in results),
            "raw_base_rinex_reads": sum(r["input_read_counts"]["base.obs"] for r in results),
            "native_process_invocations": sum(r["native_process_invocations"] for r in results),
            "truth_reads": 0,
            "solution_coordinate_reads": 0,
            "accuracy_calculations": 0,
            "mat_pdc_precomputed_coordinate_reads": 0,
            "kaggle_or_token_access": 0,
            "reruns_fallbacks_repairs_sweeps": 0,
        },
        "solution_policy": "opaque hash/bytes/newline seal only; no solution fields parsed",
        "route_failure_independence": True,
        "rerun_or_fallback": False,
    }
    RESULT.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--verify-authorization", action="store_true",
        help="verify pins and synthetic argv/help only; never opens route payloads",
    )
    parser.add_argument(
        "--execute", action="store_true",
        help="consume the independent authorization and run H then U once",
    )
    args = parser.parse_args(argv)
    if args.execute and args.verify_authorization:
        parser.error("--execute and --verify-authorization are mutually exclusive")
    try:
        auth = read_object(AUTH, "Phase147 authorization")
        plan = validate_authorization(auth)
        if args.verify_authorization:
            help_result = validate_help_argv()
            print(json.dumps({
                "status": "authorization-and-help-verified-no-payload-read",
                "routes": list(TARGETS),
                "synthetic_commands_validated": True,
                "raw_payload_reads": 0,
                "solver_invocations": 0,
                "help": help_result,
            }, sort_keys=True))
            return 0
        if not args.execute:
            print(json.dumps({
                "status": "authorization-verified-no-payload-read",
                "routes": list(TARGETS),
                "raw_payload_reads": 0,
                "solver_invocations": 0,
            }, sort_keys=True))
            return 0
        result = execute(auth)
        print(json.dumps({
            "status": result["status"],
            "route_statuses": [item.get("status") for item in result["routes"]],
            "failure_stages": [item.get("failure_stage") for item in result["routes"]],
            "read_accounting": result["read_accounting"],
        }, sort_keys=True))
        return 0
    except Phase147AuthorizationError as exc:
        print(f"PHASE147_DIAGNOSTIC_FAIL_CLOSED: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
