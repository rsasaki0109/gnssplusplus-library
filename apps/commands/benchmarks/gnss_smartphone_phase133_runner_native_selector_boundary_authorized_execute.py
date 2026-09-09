#!/usr/bin/env python3
"""Execute the independently authorized Phase133 structural matrix once.

This adapter uses the already-qualified Phase131 typed inventory and native
summary normalizer, but supplies the Phase133 command boundary.  The
Phase130 comparison selector is never placed in native argv.  Authorization
and all static pins are verified before any route payload is opened.  The
native solution is sealed only as an opaque hash and expected row count; no
solution coordinate is parsed or published.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

import gnss_smartphone_phase133_runner_native_selector_boundary as contract  # noqa: E402
import gnss_smartphone_phase131_canonical_correction_structural_authorized_execute as implementation  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_raw_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase133-runner-native-selector-boundary-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "52d397880baea7854919d00bb69c39900af5b48d"
FREEZE_COMMIT = "8b1f75be0aaaadaedfffc5671be706dabd1106043"
IMPLEMENTATION_COMMIT = "f32e0132aa144cd583a93799f320779c9e399a76"
TESTS_COMMIT = "8a20e02b1625315c12ad6bc578a253801c1a759a"
MANIFEST_COMMIT = "2ff98c1d1b751b1405fcc4c83e7f9893650672a4"
PRE_RAW_COMMIT = "3e086ac6907e049c760071919e3a5533d336cec1"
QUALIFICATION_COMMIT = "7e3e7b17fd7366a622daae46325f5e9dc9758bde"
AUDIT_SHA256 = "43ba702f45f446e0c2268b6a638fd3972d87e14812ce9adc9409e82b979a6bad"
FREEZE_SHA256 = "4dbf7bff3d9dc2537fd5fd88bf930d0c88a832bdd7f655dc47c4f5ed6b6110f6"
MANIFEST_SHA256 = "ca90d461759589c8cab7df6a328f9ac6f450f77edb6ec85fda6bbaae90161a2f"
PRE_RAW_SHA256 = "f1f17573e12527aa6ac96b811d2d7448f83e0b992ac1586b0805c71312cb156d"
TARGET_BINARY_SHA256 = "ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e"

ROUTES = contract.ROUTES
RAW_NAMES = contract.RAW_NAMES
BASE_NAME = "base.obs"
PROBLEM_EPOCHS = {
    ROUTES[0]: 2159,
    ROUTES[1]: 1466,
}


class Phase133ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase133ExecutionError:
    return Phase133ExecutionError(message)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def assert_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise fail(f"{label}: expected {expected!r}, got {actual!r}")


def read_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"failed to read {label}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object")
    return value


def preserve_native_summary(summary_path: Path,
                            native_summary_path: Path) -> dict[str, Any]:
    """Preserve the byte-exact native view before writing a normalized view.

    The native process owns ``summary_path``.  A wrapper may derive a
    structural view, but it must never destroy that source artifact.  The
    destination is create-only so an accidental second preservation attempt
    fails closed instead of silently replacing an earlier native report.
    """
    if summary_path.resolve() == native_summary_path.resolve():
        raise fail("native and normalized summary paths must be distinct")
    try:
        native_bytes = summary_path.read_bytes()
        with native_summary_path.open("xb") as handle:
            handle.write(native_bytes)
    except (OSError, ValueError) as exc:
        raise fail(f"failed to preserve native summary: {exc}") from exc
    return {
        "path": relative(native_summary_path),
        "bytes": len(native_bytes),
        "sha256": hashlib.sha256(native_bytes).hexdigest(),
        "byte_exact": True,
        "view": "raw-native",
    }


def describe_native_summary(native_summary_path: Path) -> dict[str, Any]:
    """Describe an already separate native summary without normalizing it."""
    try:
        native_bytes = native_summary_path.read_bytes()
    except (OSError, ValueError) as exc:
        raise fail(f"failed to read native summary metadata: {exc}") from exc
    return {
        "path": relative(native_summary_path),
        "bytes": len(native_bytes),
        "sha256": hashlib.sha256(native_bytes).hexdigest(),
        "byte_exact": True,
        "view": "raw-native",
    }


def static_sha(path: Path, label: str) -> str:
    """Hash static artifacts only; reject payload-looking paths."""
    lowered = path.name.lower()
    if (lowered in set(RAW_NAMES) | {BASE_NAME, "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash attempted before authorization: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def zero_accounting(value: Any, label: str) -> None:
    if not isinstance(value, dict):
        raise fail(f"{label} missing")
    for key, item in value.items():
        if isinstance(item, bool):
            assert_equal(item, False, f"{label}/{key}")
        elif isinstance(item, str):
            if item not in {"read-only", "sealed-metadata-only", "not-run"}:
                raise fail(f"{label}/{key} is not zero-read metadata: {item!r}")
        else:
            assert_equal(item, 0, f"{label}/{key}")


def verify_authorization() -> dict[str, Any]:
    """Verify every static pin without opening any route payload."""
    # These launch-free checks read source and sealed metadata only.
    contract.verify_pre_raw()
    assert_equal(static_sha(AUTHORIZED_RUNNER, "Phase133 authorized runner"),
                 _expected_runner_sha_from_auth(),
                 "authorized runner/sha256")
    auth = read_json(AUTHORIZATION, "Phase133 independent raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase133-runner-native-selector-boundary-raw-authorization.v1",
        "phase": 133,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    expected_pins = {
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "tests_commit": TESTS_COMMIT,
        "manifest_commit": MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "qualification_commit": QUALIFICATION_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_sha256": PRE_RAW_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "Phase133 authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    runner_commit = pins.get("authorized_runner_commit")
    if (not isinstance(runner_commit, str) or len(runner_commit) != 40
            or any(char not in "0123456789abcdef" for char in runner_commit)):
        raise fail("authorization/pins/authorized_runner_commit must be full lowercase SHA")
    assert_equal(pins.get("target_binary_path"), relative(contract.BINARY),
                 "authorization/pins/target_binary_path")

    scope = auth.get("authorization")
    if not isinstance(scope, dict):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization",
                "inventory_stage", "raw_structural_execution", "solver"):
        assert_equal(scope.get(key), True, f"authorization/authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair"):
        assert_equal(scope.get(key), False, f"authorization/authorization/{key}")
    scope_meta = auth.get("authorization_scope")
    if not isinstance(scope_meta, dict):
        raise fail("authorization_scope missing")
    for key, expected in {
        "candidate_id": contract.CANDIDATE_ID,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "solution_policy": "opaque SHA-256 and expected row count only",
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    selectors = auth.get("selectors")
    if not isinstance(selectors, dict):
        raise fail("authorization/selectors missing")
    expected_selectors = {
        "phase118": 1,
        "phase126": 1,
        "phase127": 1,
        "phase128": 1,
        "phase129": 1,
        "phase130": 0,
        "phase131": 1,
        "phase117": 0,
        "phase120": 0,
        "additional_frequency": 0,
    }
    assert_equal(selectors, expected_selectors, "authorization/selectors")

    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTES or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        command = record.get("command")
        contract.validate_command(route, command)
        if command.count(contract.PHASE130_SELECTOR) != 0:
            raise fail(f"authorization/{route}: Phase130 token forwarded")
        raw = record.get("raw_inputs")
        if not isinstance(raw, dict) or set(raw) != set(RAW_NAMES):
            raise fail(f"authorization/{route}/raw_inputs must be GNSS/IMU/nav only")
        for name in RAW_NAMES:
            item = raw.get(name)
            if not isinstance(item, dict):
                raise fail(f"authorization/{route}/{name} missing")
            for key in ("path", "bytes", "sha256"):
                if key not in item:
                    raise fail(f"authorization/{route}/{name}/{key} missing")
            assert_equal(item.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(item.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        if not isinstance(base, dict):
            raise fail(f"authorization/{route}/base_input missing")
        for key in ("path", "bytes", "sha256"):
            if key not in base:
                raise fail(f"authorization/{route}/base/{key} missing")
        for key, expected in {
            "read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        }.items():
            assert_equal(base.get(key), expected,
                         f"authorization/{route}/base/{key}")
    accounting = auth.get("read_accounting_before_authorization")
    zero_accounting(accounting, "authorization/read_accounting_before_authorization")
    return auth


def _expected_runner_sha_from_auth() -> str:
    """Read only the authorization pin needed before payload access."""
    if not AUTHORIZATION.is_file():
        raise fail(f"missing independent authorization: {AUTHORIZATION}")
    value = read_json(AUTHORIZATION, "Phase133 independent raw authorization")
    pins = value.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    digest = pins.get("authorized_runner_sha256")
    if not isinstance(digest, str) or len(digest) != 64:
        raise fail("authorization/pins/authorized_runner_sha256 missing")
    return digest


def command_for(route: str, auth_record: dict[str, Any],
               native_summary_path: Path | None = None) -> list[str]:
    """Substitute authorized paths into the fixed Phase133 argv only."""
    command = contract.command_template(route)
    contract.validate_command(route, command)
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        command[command.index(flag) + 1] = auth_record["raw_inputs"][name]["path"]
    command[command.index("--native-base-rinex") + 1] = auth_record[
        "base_input"]["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = auth_record[
        "base_input"]["sha256"]
    if any(token == contract.PHASE130_SELECTOR for token in command):
        raise fail("Phase130 runner-only selector crossed native argv boundary")
    for selector in contract.NATIVE_SELECTORS:
        assert_equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in contract.OFF_SELECTORS:
        assert_equal(command.count(selector), 0, f"command/{route}/{selector}")
    if native_summary_path is not None:
        summary_value = str(native_summary_path)
        if any(term in summary_value.lower()
               for term in contract.FORBIDDEN_PATH_TERMS):
            raise fail(f"forbidden native summary path: {summary_value}")
        # The contract snapshot remains the historical placeholder command;
        # the execution adapter gives the native process its own path and
        # reserves the snapshot path for the normalized structural view.
        command[command.index("--summary-json") + 1] = summary_value
    return command


def _phase133_evidence(inventory: dict[str, Any], *, native: bool,
                       resolver_calls: int = 0) -> dict[str, Any]:
    phase131 = inventory.get("phase131", {})
    return {
        "candidate_id": contract.CANDIDATE_ID,
        "typed_preflight_call_count": int(
            phase131.get("python_preflight_call_count", 1) or 1),
        "old_literal_phase130_preflight_call_count": 0,
        "native_command_constructed": native,
        "native_command_construction_count": 1 if native else 0,
        "native_selector_forwarded": native,
        "native_selector_forwarding_count": 1 if native else 0,
        "native_binary_invocation_attempted": native,
        "native_solver_invocations": 1 if native else 0,
        "native_resolver_executed": native and resolver_calls > 0,
        "native_resolver_call_count": resolver_calls,
        "native_resolver_evidence_source": "native-summary" if native else "not-launched",
        "phase130_argv_count": 0,
        "native_selector_counts": {selector: 1 for selector in contract.NATIVE_SELECTORS},
        "off_selector_counts": {selector: 0 for selector in contract.OFF_SELECTORS},
    }


def inventory_failure_record(route: str, failure: str,
                             reads: dict[str, int]) -> dict[str, Any]:
    return {
        "route": route,
        "stage": "post-independent-authorization-pre-solver",
        "ok": False,
        "solver_invocations": 0,
        "solver_may_start": False,
        "failure": failure,
        "inventory_reads": dict(reads),
        "phase131": {
            "enabled": True,
            "python_preflight_started": True,
            "python_preflight_executed": False,
            "python_preflight_call_count": 1,
        },
        "phase133": _phase133_evidence({}, native=False),
    }


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase133_inventory_handoff": False,
        "phase133_phase130_argv_absent": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "native_phase131_resolver_reached": False,
        "no_fallback_or_publication": False,
    }


def execute_one_route(route: str, auth_record: dict[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    summary_path = route_dir / "structural_summary.json"
    native_summary_path = route_dir / "native_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    command = command_for(route, auth_record, native_summary_path)
    inventory["phase133"] = _phase133_evidence(inventory, native=True)
    inventory["phase131"]["native_command_constructed"] = True
    inventory["phase131"]["native_selector_forwarded"] = True
    inventory["phase131"]["native_binary_invocation_attempted"] = True
    inventory["phase131"]["native_resolver_executed"] = False
    inventory["phase131"]["native_resolver_call_count"] = 0
    inventory["phase131"]["native_resolver_evidence_source"] = "pending-native-summary"
    implementation.atomic_json(route_dir / "inventory.json", inventory)
    return_code: int | None = None
    launch_error = ""
    env = os.environ.copy()
    env.update({"LANG": "C", "LC_ALL": "C", "TZ": "UTC"})
    env["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib" + (
        ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else "")
    with stdout_path.open("wb") as stdout, stderr_path.open("wb") as stderr:
        try:
            completed = subprocess.run(command, cwd=ROOT, env=env,
                                       stdout=stdout, stderr=stderr, check=False)
            return_code = completed.returncode
        except OSError as exc:
            launch_error = str(exc)
            stderr.write(f"Phase133 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route,
        "run_number": 1,
        "solver_launched": True,
        "return_code": return_code,
        "launch_error": launch_error,
        "summary_present": native_summary_path.is_file(),
        "native_summary_path": relative(native_summary_path),
        "normalized_summary_path": relative(summary_path),
        "native_summary_preserved": False,
        "solution_opened": False,
        "solution_published": False,
        "truth_used": False,
        "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not native_summary_path.is_file():
        record.update({"summary_error": "native summary absent; structural gates fail closed",
                       "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["phase133_phase130_argv_absent"] = True
        record["gates"]["no_fallback_or_publication"] = True
        return record
    record["summary_present"] = True
    try:
        native_summary_artifact = describe_native_summary(native_summary_path)
        record["native_summary_preserved"] = True
        record["native_summary_sha256"] = native_summary_artifact["sha256"]
        record["native_summary_bytes"] = native_summary_artifact["bytes"]
        native = read_json(native_summary_path,
                           f"Phase133 native summary {route}")
        solution = implementation.opaque_solution_seal(solution_path,
                                                        PROBLEM_EPOCHS[route])
        normalized, telemetry = implementation.normalize_native(
            route, native, solution, return_code, inventory)
        execution_evidence = telemetry["execution_evidence"]
        resolver_calls = int(execution_evidence.get("native_resolver_call_count", 0) or 0)
        inventory["phase133"] = _phase133_evidence(
            inventory, native=True, resolver_calls=resolver_calls)
        inventory["phase131"]["native_resolver_executed"] = resolver_calls > 0
        inventory["phase131"]["native_resolver_call_count"] = resolver_calls
        inventory["phase131"]["native_resolver_evidence_source"] = "native-summary"
        telemetry["phase133"] = inventory["phase133"]
        telemetry["execution_evidence"]["phase130_argv_count"] = 0
        telemetry["execution_evidence"]["native_selector_counts"] = {
            selector: 1 for selector in contract.NATIVE_SELECTORS}
        telemetry["execution_evidence"]["off_selector_counts"] = {
            selector: 0 for selector in contract.OFF_SELECTORS}
        normalized["summary_views"] = {
            "native": native_summary_artifact,
            "normalized": {
                "path": relative(summary_path),
                "view": "normalized-structural",
                "source_native_path": relative(native_summary_path),
            },
        }
        telemetry["summary_views"] = normalized["summary_views"]
        atomic = all(normalized["gates"].get(key) is True for key in (
            "phase126_atomic_a_b_c", "base_exactly_once"))
        normalized["gates"]["phase133_inventory_handoff"] = inventory.get("ok") is True
        normalized["gates"]["phase133_phase130_argv_absent"] = True
        normalized["gates"]["native_phase131_resolver_reached"] = resolver_calls > 0
        normalized["gates"]["phase126_atomic_a_b_c"] = atomic
        implementation.atomic_json(route_dir / "inventory.json", inventory)
        implementation.atomic_json(summary_path, normalized)
    except (implementation.Phase131ExecutionError, KeyError, TypeError, ValueError) as exc:
        record.update({"summary_error": str(exc), "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["summary_present"] = True
        record["gates"]["phase133_phase130_argv_absent"] = True
        record["gates"]["no_fallback_or_publication"] = True
        return record
    record.update({
        "summary_schema": normalized["schema_version"],
        "summary_status": native.get("status"),
        "solution_present": solution.get("present", False),
        "solution_hash_sealed": solution.get("sha256"),
        "solution_rows_sealed": solution.get("rows"),
        "solution_hash_only": True,
        "structural_telemetry": telemetry,
        "gates": normalized["gates"],
    })
    return record


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase133 runner/native selector boundary structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one inventory pass and at most one native solver attempt per route.",
        "- Phase130 is runner-only and is absent from every native argv; native selectors 126/127/128/129/131/118 are each forwarded once.",
        "- The byte-exact native summary is preserved as `native_summary.json`; `structural_summary.json` is the separate normalized view.",
        "- Solution rows were not opened or interpreted; only opaque hashes and expected row counts were sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.",
        "",
        "| Route | Inventory | Solver | Return | Typed calls | Old literal calls | Phase130 argv | Resolver calls | Main accepted | Main cost | GO | Failure |",
        "|---|---|---|---:|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result.get("routes", {}).get(route, {})
        inventory = record.get("inventory", {})
        phase = inventory.get("phase133", {})
        main = record.get("structural_telemetry", {}).get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | "
            f"`{record.get('return_code')}` | `{phase.get('typed_preflight_call_count', 0)}` | `0` | "
            f"`{phase.get('phase130_argv_count', 0)}` | `{phase.get('native_resolver_call_count', 0)}` | "
            f"`{main.get('accepted_iterations', 0)}` | `{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend([
        "",
        "Phase133 read accounting records no truth/accuracy/MAT/PDC/precomputed-coordinate/Kaggle access.",
        "Structural failure is fail-closed; no rerun, fallback, repair, or solution publication is permitted.",
        "",
    ])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    auth = verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists() or OUTPUT_ROOT.exists():
        raise fail("refusing to overwrite an existing Phase133 result/output")
    accounting: dict[str, Any] = {
        "raw_device_gnss_inventory_reads": 0,
        "raw_device_imu_inventory_reads": 0,
        "broadcast_navigation_inventory_reads": 0,
        "raw_base_rinex_inventory_reads": 0,
        "raw_base_header_inventory_reads": 0,
        "raw_base_hash_reads": 0,
        "native_command_constructions": 0,
        "native_solver_invocations": 0,
        "solution_rows_opened": 0,
        "solution_opaque_hash_reads": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "mat_reads_or_generated": 0,
        "pdc_reads": 0,
        "precomputed_coordinate_reads": 0,
        "accuracy_calculations": 0,
        "kaggle_or_token_access": 0,
        "route_reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "raw_content_copied_or_transformed": False,
    }
    route_records: dict[str, Any] = {}
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=False)
    for route in ROUTES:
        auth_record = next(item for item in auth["routes"] if item["dataset_id"] == route)
        payloads: dict[str, bytes] = {}
        reads: dict[str, int] = {}
        try:
            for name in RAW_NAMES:
                path = implementation.safe_payload_path(
                    auth_record["raw_inputs"][name]["path"], name, route)
                payloads[name] = implementation.read_payload_once(
                    path, auth_record["raw_inputs"][name], route, name, reads)
                accounting[{"device_gnss.csv": "raw_device_gnss_inventory_reads",
                             "device_imu.csv": "raw_device_imu_inventory_reads",
                             "brdc.nav": "broadcast_navigation_inventory_reads"}[name]] += 1
            base_path = implementation.safe_payload_path(
                auth_record["base_input"]["path"], BASE_NAME, route)
            payloads[BASE_NAME] = implementation.read_payload_once(
                base_path, auth_record["base_input"], route, BASE_NAME, reads)
            accounting["raw_base_rinex_inventory_reads"] += 1
            accounting["raw_base_header_inventory_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = implementation.build_inventory(route, payloads, reads)
        except (implementation.Phase131ExecutionError, KeyError, TypeError, ValueError) as exc:
            inventory = inventory_failure_record(route, str(exc), reads)
        if inventory.get("ok") is True:
            route_record = execute_one_route(route, auth_record, inventory)
            accounting["native_command_constructions"] += 1
            accounting["native_solver_invocations"] += 1
            if route_record.get("solution_present"):
                accounting["solution_opaque_hash_reads"] += 1
        else:
            route_dir = OUTPUT_ROOT / route.replace("/", "__")
            route_dir.mkdir(parents=True, exist_ok=False)
            implementation.atomic_json(route_dir / "inventory.json", inventory)
            route_record = {
                "route": route,
                "run_number": 1,
                "solver_launched": False,
                "return_code": None,
                "solution_opened": False,
                "solution_published": False,
                "truth_used": False,
                "accuracy_scored": False,
                "raw_content_copied_or_transformed": False,
                "inventory": inventory,
                "failure": "pre-solver inventory failed closed",
                "gates": empty_gates(),
            }
            route_record["gates"]["phase133_phase130_argv_absent"] = True
            route_record["gates"]["no_fallback_or_publication"] = True
        route_records[route] = route_record
        implementation.atomic_json(OUTPUT_ROOT / "partial_result.json", {
            "routes": route_records,
            "completed_routes": list(route_records),
            "native_solver_invocations": accounting["native_solver_invocations"],
            "phase133_phase130_argv_count": 0,
        })
        del payloads
    all_passed = all(all(route_records[route].get("gates", {}).values()) for route in ROUTES)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase133-runner-native-selector-boundary-structural-result.v1",
        "phase": 133,
        "execution_label": "Luna Max",
        "status": "go-phase133-runner-native-selector-boundary-structural" if all_passed
                  else "no-go-phase133-runner-native-selector-boundary-structural",
        "decision": ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth.get("status"), "independent": True},
        "contract": {
            "candidate_id": contract.CANDIDATE_ID,
            "audit_commit": AUDIT_COMMIT,
            "freeze_commit": FREEZE_COMMIT,
            "implementation_commit": IMPLEMENTATION_COMMIT,
            "tests_commit": TESTS_COMMIT,
            "manifest_commit": MANIFEST_COMMIT,
            "pre_raw_commit": PRE_RAW_COMMIT,
            "qualification_commit": QUALIFICATION_COMMIT,
            "manifest_sha256": MANIFEST_SHA256,
            "target_binary_sha256": TARGET_BINARY_SHA256,
            "authorized_runner_path": relative(AUTHORIZED_RUNNER),
        },
        "candidate": {
            "id": contract.CANDIDATE_ID,
            "phase118": True,
            "phase126": True,
            "phase127": True,
            "phase128": True,
            "phase129": True,
            "phase130_native_argv": False,
            "phase131": True,
            "phase117_dynamic_sigma": False,
            "phase120_atmosphere": False,
            "additional_frequency_bands": False,
            "fixed_tdcp_sigma": 0.03,
            "official_huber_k": 0.5,
            "main_solver": "MULTIFRONTAL_QR",
            "solution_opaque": True,
        },
        "selector_ownership": contract.selector_ownership(),
        "execution_evidence": {
            "typed_preflight_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase133", {}).get(
                    "typed_preflight_call_count", 0) > 0),
            "old_literal_phase130_preflight_calls": 0,
            "phase130_native_argv_count": 0,
            "native_command_construction_routes": accounting["native_command_constructions"],
            "native_solver_invocation_routes": accounting["native_solver_invocations"],
            "native_resolver_reached_routes": sum(
                1 for item in route_records.values()
                if item.get("inventory", {}).get("phase133", {}).get(
                    "native_resolver_executed") is True),
            "native_resolver_call_count": sum(
                int(item.get("inventory", {}).get("phase133", {}).get(
                    "native_resolver_call_count", 0) or 0)
                for item in route_records.values()),
            "native_selector_counts_per_passing_route": {
                selector: 1 for selector in contract.NATIVE_SELECTORS},
            "off_selector_counts_per_route": {
                selector: 0 for selector in contract.OFF_SELECTORS},
            "native_resolver_evidence_source": "native-summary-only",
        },
        "matrix": {
            "candidate_count": 1,
            "route_order": ["MTV-A", "LAX-T"],
            "runs_per_route": 1,
            "native_solver_invocations": accounting["native_solver_invocations"],
            "controls": 0,
            "reruns": 0,
            "fallbacks": 0,
            "truth_reads": 0,
            "accuracy_calculations": 0,
            "solution_rows_opened": 0,
        },
        "routes": route_records,
        "read_accounting": accounting,
        "truth_free": True,
        "accuracy_scored": False,
        "solution_output_published": False,
        "promotion_authorized": False,
        "stop_before_truth_accuracy_submission": True,
    }
    implementation.atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify static pins without opening route payloads")
    parser.add_argument("--execute", action="store_true",
                        help="run exactly the authorized MTV-A then LAX-T matrix")
    args = parser.parse_args()
    if args.verify_authorization and args.execute:
        parser.error("verification and execution are separate modes")
    if not (args.verify_authorization or args.execute):
        parser.error("one mode is required")
    try:
        if args.verify_authorization:
            verify_authorization()
            print(json.dumps({"status": "authorization-pins-verified",
                              "raw_reads": 0, "solver_invocations": 0}, sort_keys=True))
            return 0
        result = execute_matrix()
        print(json.dumps({
            "status": result["status"],
            "routes": {route: {
                "inventory_ok": result["routes"][route]["inventory"].get("ok"),
                "solver_launched": result["routes"][route].get("solver_launched"),
                "return_code": result["routes"][route].get("return_code"),
                "typed_preflight_calls": result["routes"][route]["inventory"].get(
                    "phase133", {}).get("typed_preflight_call_count", 0),
                "old_literal_preflight_calls": 0,
                "phase130_argv_count": result["routes"][route]["inventory"].get(
                    "phase133", {}).get("phase130_argv_count", 0),
                "native_resolver_calls": result["routes"][route]["inventory"].get(
                    "phase133", {}).get("native_resolver_call_count", 0),
                "main_iterations": result["routes"][route].get("structural_telemetry", {})
                    .get("main", {}).get("accepted_iterations", 0),
                "main_cost": [result["routes"][route].get("structural_telemetry", {})
                               .get("main", {}).get("initial_cost"),
                               result["routes"][route].get("structural_telemetry", {})
                               .get("main", {}).get("final_cost")],
                "failure": result["routes"][route].get("failure") or
                           result["routes"][route]["inventory"].get("failure"),
            } for route in ROUTES},
            "read_accounting": result["read_accounting"],
        }, indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase133ExecutionError, implementation.Phase131ExecutionError,
            contract.Phase133ContractError, OSError) as exc:
        print(f"phase133 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
