#!/usr/bin/env python3
"""Execute the independently authorized Phase134 structural matrix once.

Authorization pins and route metadata are verified before any raw payload is
opened.  The route order is fixed to MTV-A then LAX-T, with one inventory
pass and at most one native invocation per route.  The native process writes
``native_summary.json`` directly; this wrapper writes the normalized
``structural_summary.json`` separately and never overwrites the native view.
Solution bytes are sealed only as opaque hash/row metadata.  Truth,
accuracy, MAT/PDC/precomputed coordinates, Kaggle, retries, fallback, and
repair remain outside this authorization.
"""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

import gnss_smartphone_phase131_canonical_correction_structural_authorized_execute as implementation  # noqa: E402
import gnss_smartphone_phase134_native_summary_bridge as contract  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_raw_authorization_v1.json"
REFERENCE_AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase133_runner_native_selector_boundary_raw_authorization_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase134-native-summary-bridge-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase134_native_summary_bridge_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "a116556d2437f039f0d6947b2b4b445928d64af8"
FREEZE_COMMIT = "7d00f6e0a43d1b4a8c504db9705a3fa34e67b4cc"
IMPLEMENTATION_COMMIT = "130a7f8f12e191cc12ebd0ff66ad591d732775f7"
MANIFEST_COMMIT = "1ed9d906ae395a4ac1c530d7f446e6ba47ccd7f5"
PRE_RAW_COMMIT = "041a36aaf8cac83f36c2e55396a1a6c5fbbf59b7"
AUDIT_SHA256 = "d5e62de1e4ad4b2c6219428e5c58a19f0f4b12549a8f0e9b8a27c04626aaa1a3"
FREEZE_SHA256 = "506c0433556c62f17fdf408e8e34b35238e0b8c579f4dbd796867ffee97e8dbb"
MANIFEST_SHA256 = "7d0074c8b93955018f4023032c8ac86135bed87a68a1b5841af4cc5388ac5e72"
PRE_RAW_SHA256 = "6506f4643e3c3a2eba0b8743e541187765f96e19ea1386f37c9aef05598eac59"
TARGET_BINARY_SHA256 = "3965852271023671cd0e9c6ea3e779ab6f67f4e883d724ccacadf28bd8b2fb79"
LAUNCH_FREE_RUNNER_SHA256 = "4b19e13fae965721895bb34f51554ae7d7b6231426a1ae8a51ec977c1a7e9e47"

ROUTES = contract.ROUTES
RAW_NAMES = contract.RAW_NAMES
BASE_NAME = "base.obs"
PROBLEM_EPOCHS = {ROUTES[0]: 2159, ROUTES[1]: 1466}


class Phase134ExecutionError(ValueError):
    """Authorization, inventory, or structural execution failure."""


def fail(message: str) -> Phase134ExecutionError:
    return Phase134ExecutionError(message)


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


def static_sha(path: Path, label: str) -> str:
    """Hash only a static artifact; reject payload-looking file names."""
    lowered = path.name.lower()
    if (lowered in set(RAW_NAMES) | {BASE_NAME, "truth.csv", "ground_truth.csv"}
            or lowered.endswith((".csv", ".nav", ".obs", ".mat"))
            or "truth" in lowered or "ground_truth" in lowered):
        raise fail(f"payload hash attempted as static artifact: {label}")
    if not path.is_file():
        raise fail(f"missing static artifact: {label}: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def zero_accounting(value: Any, label: str) -> None:
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


def verify_authorization() -> dict[str, Any]:
    """Verify all static pins and route metadata before raw reads."""
    contract.verify_pre_raw()
    contract.verify_static_sources()
    auth = read_json(AUTHORIZATION, "Phase134 independent raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-raw-authorization.v1",
        "phase": 134,
        "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, Mapping):
        raise fail("authorization/pins missing")
    expected_pins = {
        "audit_commit": AUDIT_COMMIT,
        "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "manifest_commit": MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256,
        "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256,
        "pre_raw_sha256": PRE_RAW_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "launch_free_runner_path": relative(contract.LAUNCH_FREE_RUNNER),
        "launch_free_runner_sha256": LAUNCH_FREE_RUNNER_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "Phase134 authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    runner_commit = pins.get("authorized_runner_commit")
    if runner_commit is not None and (
            not isinstance(runner_commit, str) or len(runner_commit) != 40 or
            any(char not in "0123456789abcdef" for char in runner_commit)):
        raise fail("authorization/pins/authorized_runner_commit is not a full SHA")

    scope = auth.get("authorization")
    if not isinstance(scope, Mapping):
        raise fail("authorization/authorization missing")
    for key in ("implementation", "contract", "raw_materialization",
                "inventory_stage", "raw_structural_execution", "solver"):
        assert_equal(scope.get(key), True, f"authorization/authorization/{key}")
    for key in ("truth_evaluation", "accuracy", "solution_publication",
                "kaggle_submission", "rerun", "fallback", "repair", "sweep"):
        assert_equal(scope.get(key), False, f"authorization/authorization/{key}")
    scope_meta = auth.get("authorization_scope")
    if not isinstance(scope_meta, Mapping):
        raise fail("authorization_scope missing")
    for key, expected in {
        "candidate_id": contract.CANDIDATE_ID,
        "route_order": ["MTV-A", "LAX-T"],
        "runs_per_route": 1,
        "controls": 0,
        "reruns": 0,
        "fallbacks": 0,
        "repairs": 0,
        "sweeps": 0,
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    selectors = auth.get("selectors")
    if not isinstance(selectors, Mapping):
        raise fail("authorization/selectors missing")
    expected_selectors = {
        "phase118": 1, "phase126": 1, "phase127": 1, "phase128": 1,
        "phase129": 1, "phase130": 0, "phase131": 1,
        "phase117": 0, "phase120": 0, "additional_frequency": 0,
    }
    for key, expected in expected_selectors.items():
        assert_equal(selectors.get(key), expected, f"authorization/selectors/{key}")

    reference = read_json(REFERENCE_AUTHORIZATION, "sealed Phase133 route metadata")
    reference_routes = {item.get("dataset_id"): item for item in reference.get("routes", [])}
    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in reference_routes or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        reference_record = reference_routes[route]
        raw = record.get("raw_inputs")
        reference_raw = reference_record.get("raw_inputs")
        if not isinstance(raw, Mapping) or not isinstance(reference_raw, Mapping):
            raise fail(f"authorization/{route}/raw_inputs missing")
        for name in RAW_NAMES:
            actual = raw.get(name)
            expected = reference_raw.get(name)
            if not isinstance(actual, Mapping) or not isinstance(expected, Mapping):
                raise fail(f"authorization/{route}/{name} missing")
            for key in ("path", "bytes", "sha256"):
                assert_equal(actual.get(key), expected.get(key),
                             f"authorization/{route}/{name}/{key}")
            assert_equal(actual.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(actual.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        reference_base = reference_record.get("base_input")
        if not isinstance(base, Mapping) or not isinstance(reference_base, Mapping):
            raise fail(f"authorization/{route}/base_input missing")
        for key in ("path", "bytes", "sha256", "interval", "window"):
            assert_equal(base.get(key), reference_base.get(key),
                         f"authorization/{route}/base/{key}")
        for key, expected in {
            "read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        }.items():
            assert_equal(base.get(key), expected,
                         f"authorization/{route}/base/{key}")
        command = record.get("command_template")
        if command != "manifest-pinned":
            raise fail(f"authorization/{route}/command_template must reference manifest")
        contract.validate_command(route, contract.command_template(route))
        summary = record.get("summary_artifacts")
        if not isinstance(summary, Mapping):
            raise fail(f"authorization/{route}/summary_artifacts missing")
        assert_equal(summary.get("native_view_filename"), "native_summary.json",
                     f"authorization/{route}/summary/native")
        assert_equal(summary.get("normalized_view_filename"), "structural_summary.json",
                     f"authorization/{route}/summary/normalized")
        assert_equal(summary.get("native_overwrite_allowed"), False,
                     f"authorization/{route}/summary/overwrite")
    zero_accounting(auth.get("read_accounting_before_authorization"),
                    "authorization/read_accounting_before_authorization")
    return auth


def command_for(route: str, auth_record: Mapping[str, Any],
                native_summary_path: Path) -> list[str]:
    command = contract.command_template(route)
    contract.validate_command(route, command)
    for flag, name in (("--android-gnss", "device_gnss.csv"),
                       ("--android-imu", "device_imu.csv"),
                       ("--nav", "brdc.nav")):
        value = auth_record["raw_inputs"][name]["path"]
        if not isinstance(value, str):
            raise fail(f"{route}/{name}: missing authorized path")
        command[command.index(flag) + 1] = value
    base = auth_record["base_input"]
    command[command.index("--native-base-rinex") + 1] = base["path"]
    command[command.index("--native-base-rinex-sha256") + 1] = base["sha256"]
    if native_summary_path.is_absolute():
        summary_value = str(native_summary_path)
    else:
        summary_value = str(ROOT / native_summary_path)
    if any(term in summary_value.lower() for term in contract.FORBIDDEN_PATH_TERMS):
        raise fail(f"forbidden native summary path: {summary_value}")
    command[command.index("--summary-json") + 1] = summary_value
    if command.count(contract.PHASE130_SELECTOR) != 0:
        raise fail("Phase130 crossed native argv boundary")
    for selector in contract.NATIVE_SELECTORS:
        assert_equal(command.count(selector), 1, f"command/{route}/{selector}")
    for selector in contract.OFF_SELECTORS:
        assert_equal(command.count(selector), 0, f"command/{route}/{selector}")
    return command


def describe_native_summary(path: Path, route: str) -> dict[str, Any]:
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise fail(f"{route}: native summary read failed: {exc}") from exc
    if not payload:
        raise fail(f"{route}: native summary is empty")
    return {
        "path": relative(path),
        "bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "source": "native-process",
        "byte_exact": True,
        "overwritten": False,
    }


def empty_gates() -> dict[str, bool]:
    return {
        "native_process_completed": False,
        "summary_present": False,
        "phase131_inventory_handoff": False,
        "bridge_source_exact": False,
        "bridge_exactly_once": False,
        "base_top_level_counter_equality": False,
        "resolver_attempt_positive": False,
        "canonical_conservation": False,
        "phase126_atomic_a_b_c": False,
        "base_exactly_once": False,
        "gnss_first_progress": False,
        "gnss_first_c7_d_handoff": False,
        "main_qr_progress": False,
        "main_finite_coverage": False,
        "native_summary_immutable": False,
        "normalized_summary_separate": False,
        "no_fallback_or_publication": False,
    }


def failure_summary(route: str, message: str, native_view: Mapping[str, Any] | None = None) -> dict[str, Any]:
    return {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-summary.v1",
        "route": route,
        "status": "structural-failure-closed",
        "failure": message,
        "solution_content_read": False,
        "fallback_used": False,
        "rerun_count": 0,
        "summary_views": {
            "native": dict(native_view) if native_view else None,
            "normalized": {
                "path": relative(OUTPUT_ROOT / route.replace("/", "__") / "structural_summary.json"),
                "source": "wrapper-normalized-view",
                "source_native_path": relative(OUTPUT_ROOT / route.replace("/", "__") / "native_summary.json"),
            },
        },
        "gates": empty_gates(),
    }


def execute_one_route(route: str, auth_record: Mapping[str, Any],
                      inventory: dict[str, Any]) -> dict[str, Any]:
    route_dir = OUTPUT_ROOT / route.replace("/", "__")
    route_dir.mkdir(parents=True, exist_ok=False)
    stdout_path = route_dir / "stdout.log"
    stderr_path = route_dir / "stderr.log"
    native_summary_path = route_dir / "native_summary.json"
    normalized_summary_path = route_dir / "structural_summary.json"
    solution_path = route_dir / "opaque_solution_output.csv"
    command = command_for(route, auth_record, native_summary_path)
    phase131 = inventory.setdefault("phase131", {})
    phase131["native_command_constructed"] = True
    phase131["native_selector_forwarded"] = True
    phase131["native_binary_invocation_attempted"] = True
    phase131["native_resolver_evidence_source"] = "pending-native-summary"
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
            stderr.write(f"Phase134 native launch failed: {exc}\n".encode())
    record: dict[str, Any] = {
        "route": route,
        "run_number": 1,
        "solver_launched": True,
        "return_code": return_code,
        "launch_error": launch_error,
        "native_summary_path": relative(native_summary_path),
        "normalized_summary_path": relative(normalized_summary_path),
        "native_summary_preserved": False,
        "native_summary_overwritten": False,
        "solution_opened": False,
        "solution_published": False,
        "truth_used": False,
        "accuracy_scored": False,
        "raw_content_copied_or_transformed": False,
        "inventory": inventory,
    }
    if not native_summary_path.is_file():
        normalized = failure_summary(route, "native summary absent; structural gates fail closed")
        implementation.atomic_json(normalized_summary_path, normalized)
        record.update({"summary_present": False, "summary_error": normalized["failure"],
                       "gates": empty_gates()})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["normalized_summary_separate"] = True
        record["gates"]["no_fallback_or_publication"] = True
        return record
    try:
        native_view = describe_native_summary(native_summary_path, route)
        native = json.loads(native_summary_path.read_text(encoding="utf-8"))
        if not isinstance(native, dict):
            raise fail(f"{route}: native summary is not an object")
        bridge_evidence = contract.validate_bridge_summary(native)
        solution = implementation.opaque_solution_seal(solution_path,
                                                        PROBLEM_EPOCHS[route])
        normalized, telemetry = implementation.normalize_native(
            route, native, solution, return_code, inventory)
        normalized_view = {
            "path": relative(normalized_summary_path),
            "source": "wrapper-normalized-view",
            "source_native_path": relative(native_summary_path),
        }
        contract.validate_summary_views(native_view, normalized_view)
        normalized["summary_views"] = {"native": native_view, "normalized": normalized_view}
        normalized["phase134"] = {
            "bridge": bridge_evidence,
            "native_summary_immutable": True,
            "native_summary_overwritten": False,
            "normalized_summary_separate": True,
            "native_summary_path": relative(native_summary_path),
            "normalized_summary_path": relative(normalized_summary_path),
        }
        telemetry["phase134"] = normalized["phase134"]
        normalized["gates"].update({
            "bridge_source_exact": True,
            "bridge_exactly_once": bridge_evidence["bridge_exactly_once"],
            "base_top_level_counter_equality": bridge_evidence["base_top_level_exact_copy"],
            "resolver_attempt_positive": bridge_evidence["resolver_call_count"] > 0,
            "canonical_conservation": bridge_evidence["canonical_conservation_valid"] and
                                      bridge_evidence["reject_reason_conservation_valid"],
            "native_summary_immutable": True,
            "normalized_summary_separate": True,
        })
        implementation.atomic_json(normalized_summary_path, normalized)
    except (implementation.Phase131ExecutionError, contract.Phase134ContractError,
            KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as exc:
        native_view = None
        try:
            native_view = describe_native_summary(native_summary_path, route)
        except Phase134ExecutionError:
            pass
        normalized = failure_summary(route, str(exc), native_view)
        implementation.atomic_json(normalized_summary_path, normalized)
        record.update({"summary_present": True, "summary_error": str(exc),
                       "native_summary_bytes": native_view.get("bytes") if native_view else None,
                       "native_summary_sha256": native_view.get("sha256") if native_view else None,
                       "gates": normalized["gates"]})
        record["gates"]["native_process_completed"] = return_code == 0 and not launch_error
        record["gates"]["native_summary_immutable"] = native_view is not None
        record["gates"]["normalized_summary_separate"] = True
        record["gates"]["no_fallback_or_publication"] = True
        return record
    record.update({
        "summary_present": True,
        "summary_schema": normalized["schema_version"],
        "summary_status": native.get("status"),
        "native_summary_bytes": native_view["bytes"],
        "native_summary_sha256": native_view["sha256"],
        "solution_present": solution.get("present", False),
        "solution_hash_sealed": solution.get("sha256"),
        "solution_rows_sealed": solution.get("rows"),
        "solution_hash_only": True,
        "structural_telemetry": telemetry,
        "gates": normalized["gates"],
    })
    return record


def result_markdown(result: Mapping[str, Any]) -> str:
    lines = [
        "# Phase134 native-summary bridge structural raw result",
        "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one authorized inventory pass and at most one native invocation per route.",
        "- Native selectors 126/127/128/129/131/118 are each forwarded once; Phase130 is absent from native argv.",
        "- Native `native_summary.json` is immutable; `structural_summary.json` is the separate normalized view.",
        "- Solution content is not interpreted; only opaque hash and expected row count are sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain forbidden.",
        "",
        "| Route | Inventory | Return | Bridge | Resolver/attempt | Canonical rows/rejects | Main accepted | Main cost | Native bytes/hash | GO | Failure |",
        "|---|---|---:|---|---:|---:|---:|---|---|---|---|",
    ]
    for route in ROUTES:
        record = result.get("routes", {}).get(route, {})
        inventory = record.get("inventory", {})
        native = record.get("structural_telemetry", {})
        phase = native.get("phase131", {})
        p134 = native.get("phase134", {})
        main = native.get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('return_code')}` | "
            f"`{p134.get('bridge', {}).get('bridge_exactly_once', False)}` | "
            f"`{phase.get('native_resolver_call_count', 0)}/{phase.get('canonicalization_attempt_rows', 0)}` | "
            f"`{phase.get('canonical_rows', 0)}/{phase.get('canonical_rejected_rows', 0)}` | "
            f"`{main.get('accepted_iterations', 0)}` | "
            f"`{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{record.get('native_summary_bytes')}/{record.get('native_summary_sha256')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend([
        "",
        "Read accounting excludes all truth/accuracy/MAT/PDC/precomputed-coordinate/Kaggle access.",
        "Structural failure is sealed fail-closed; no rerun, fallback, repair, or solution publication is permitted.",
        "",
    ])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    auth = verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists() or OUTPUT_ROOT.exists():
        raise fail("refusing to overwrite existing Phase134 result/output")
    accounting: dict[str, Any] = {
        "raw_device_gnss_reads": 0,
        "raw_device_imu_reads": 0,
        "broadcast_navigation_reads": 0,
        "raw_base_rinex_payload_reads": 0,
        "raw_base_header_reads": 0,
        "raw_base_hash_reads": 0,
        "native_command_constructions": 0,
        "native_solver_invocations": 0,
        "solution_opaque_hash_reads": 0,
        "solution_coordinate_row_reads": 0,
        "solution_coordinate_interpretations": 0,
        "truth_reads": 0,
        "accuracy_calculations": 0,
        "mat_pdc_precomputed_coordinate_reads": 0,
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
                accounting[{"device_gnss.csv": "raw_device_gnss_reads",
                             "device_imu.csv": "raw_device_imu_reads",
                             "brdc.nav": "broadcast_navigation_reads"}[name]] += 1
            base_path = implementation.safe_payload_path(
                auth_record["base_input"]["path"], BASE_NAME, route)
            payloads[BASE_NAME] = implementation.read_payload_once(
                base_path, auth_record["base_input"], route, BASE_NAME, reads)
            accounting["raw_base_rinex_payload_reads"] += 1
            accounting["raw_base_header_reads"] += 1
            accounting["raw_base_hash_reads"] += 1
            inventory = implementation.build_inventory(route, payloads, reads)
        except (implementation.Phase131ExecutionError, KeyError, TypeError, ValueError) as exc:
            inventory = {
                "route": route,
                "stage": "post-independent-authorization-pre-solver",
                "ok": False,
                "failure": str(exc),
                "solver_invocations": 0,
                "solver_may_start": False,
                "phase134": {
                    "canonical_preflight": "failed-closed",
                    "native_solver_invocations": 0,
                },
                "inventory_reads": dict(reads),
            }
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
                "native_summary_path": relative(route_dir / "native_summary.json"),
                "normalized_summary_path": relative(route_dir / "structural_summary.json"),
                "native_summary_preserved": False,
                "native_summary_overwritten": False,
                "solution_opened": False,
                "solution_published": False,
                "truth_used": False,
                "accuracy_scored": False,
                "raw_content_copied_or_transformed": False,
                "inventory": inventory,
                "failure": "pre-solver inventory failed closed",
                "gates": empty_gates(),
            }
            route_record["gates"]["normalized_summary_separate"] = True
            route_record["gates"]["no_fallback_or_publication"] = True
        route_records[route] = route_record
        implementation.atomic_json(OUTPUT_ROOT / "partial_result.json", {
            "routes": route_records,
            "completed_routes": list(route_records),
            "native_solver_invocations": accounting["native_solver_invocations"],
            "phase130_native_argv_count": 0,
        })
        del payloads
    all_passed = all(all(route_records[route].get("gates", {}).values()) for route in ROUTES)
    result: dict[str, Any] = {
        "schema_version": "smartphone-r5-phase134-native-summary-bridge-structural-result.v1",
        "phase": 134,
        "execution_label": "Luna Max",
        "status": "go-phase134-native-summary-bridge-structural" if all_passed else
                  "no-go-phase134-native-summary-bridge-structural",
        "decision": ("All bridge and existing structural gates passed; truth/accuracy and solution release remain unauthorized."
                     if all_passed else
                     "Bridge, inventory, or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted."),
        "authorization": {"path": relative(AUTHORIZATION), "status": auth["status"], "independent": True},
        "contract": {
            "candidate_id": contract.CANDIDATE_ID,
            "audit_commit": AUDIT_COMMIT,
            "freeze_commit": FREEZE_COMMIT,
            "implementation_commit": IMPLEMENTATION_COMMIT,
            "manifest_commit": MANIFEST_COMMIT,
            "pre_raw_commit": PRE_RAW_COMMIT,
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
                if item.get("inventory", {}).get("phase131", {}).get(
                    "python_preflight_call_count", 0) > 0),
            "phase130_native_argv_count": 0,
            "native_command_construction_routes": accounting["native_command_constructions"],
            "native_solver_invocation_routes": accounting["native_solver_invocations"],
            "bridge_valid_routes": sum(
                1 for item in route_records.values()
                if item.get("gates", {}).get("bridge_exactly_once") is True),
            "native_summary_overwrite_routes": 0,
            "native_summary_evidence_source": "native-process-direct-path",
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


def main(argv: Sequence[str] | None = None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify pins and route metadata without raw reads")
    parser.add_argument("--execute", action="store_true",
                        help="execute exactly MTV-A then LAX-T once")
    args = parser.parse_args(argv)
    if args.verify_authorization and args.execute:
        parser.error("verification and execution are separate modes")
    if not (args.verify_authorization or args.execute):
        parser.error("one mode is required")
    try:
        if args.verify_authorization:
            verify_authorization()
            print(json.dumps({"status": "authorization-pins-verified",
                              "raw_reads": 0,
                              "solver_invocations": 0}, sort_keys=True))
            return 0
        result = execute_matrix()
        print(json.dumps({
            "status": result["status"],
            "routes": {route: {
                "inventory_ok": result["routes"][route]["inventory"].get("ok"),
                "solver_launched": result["routes"][route].get("solver_launched"),
                "return_code": result["routes"][route].get("return_code"),
                "bridge_exactly_once": result["routes"][route].get("gates", {}).get(
                    "bridge_exactly_once", False),
                "native_summary_bytes": result["routes"][route].get("native_summary_bytes"),
                "native_summary_sha256": result["routes"][route].get("native_summary_sha256"),
                "resolver_attempt": result["routes"][route].get("structural_telemetry", {})
                    .get("phase131", {}).get("native_resolver_call_count", 0),
                "main_iterations": result["routes"][route].get("structural_telemetry", {})
                    .get("main", {}).get("accepted_iterations", 0),
                "main_cost": [result["routes"][route].get("structural_telemetry", {})
                               .get("main", {}).get("initial_cost"),
                               result["routes"][route].get("structural_telemetry", {})
                               .get("main", {}).get("final_cost")],
                "go": all(result["routes"][route].get("gates", {}).values()),
            } for route in ROUTES},
            "read_accounting": result["read_accounting"],
        }, sort_keys=True))
        return 0
    except (Phase134ExecutionError, implementation.Phase131ExecutionError,
            contract.Phase134ContractError, OSError, KeyError, TypeError, ValueError) as exc:
        print(f"Phase134 fail-closed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
