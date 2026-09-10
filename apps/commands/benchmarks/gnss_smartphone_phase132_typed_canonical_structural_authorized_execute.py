#!/usr/bin/env python3
"""Execute the independently authorized Phase132 structural matrix once.

This adapter keeps the already-qualified Phase131 raw runner as the single
native execution implementation while supplying a Phase132 authorization
boundary and result namespace.  It performs no payload read until the
authorization pins have been checked.  Inventory and native summaries are
structural metadata; solution bytes are sealed only as an opaque hash and are
never parsed or published.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))

import gnss_smartphone_phase132_typed_canonical_structural as contract  # noqa: E402
import gnss_smartphone_phase131_canonical_correction_structural_authorized_execute as implementation  # noqa: E402
import gnss_smartphone_phase130_shared_ledger_structural_authorized_execute as p130  # noqa: E402


AUTHORIZATION = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_authorization_v1.json"
PRE_RAW = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_pre_raw_accounting_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_manifest_v1.json"
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase132-typed-canonical-structural-v1"
RESULT_JSON = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_result_v1.json"
RESULT_MD = ROOT / "docs/use_cases/records/smartphone_r5_phase132_typed_canonical_structural_result_v1.md"
AUTHORIZED_RUNNER = Path(__file__)

AUDIT_COMMIT = "f6f15a5bb90a533ae5f06590cd274f005609eaca"
FREEZE_COMMIT = "75bd859ea29aa8dadc03fc53b131eacc7da6298b"
IMPLEMENTATION_COMMIT = "4f546734771ac56d8342aadb88d6c36f454540fa"
CONTRACT_CODE_COMMIT = "9c833dcaccffc079cdbb7b1e4c73876c7f5b8d70"
MANIFEST_COMMIT = "df3672fd5fb7abae733cd0aed8657162d2cdcb42"
PRE_RAW_COMMIT = "f1d28532ff2f5550a202520c3ec24c050b7c783a"
AUDIT_SHA256 = "d4afbca4a84251bd791d796624fe282529bb4c0d05a2076a8ec68fa27f3e0a82"
FREEZE_SHA256 = "2cb249efe3dee6ff22d1fb6ee48f997591ab836bf30f59aeadf85b3110e3aaa9"
MANIFEST_SHA256 = "03d8d89b9cbd544cc3cfdabc9157c51531ecc360ea95e0a66cbb5c1bc6ed372a"
PRE_RAW_SHA256 = "92cccfb1c55a43852c803b86ffc3b964fc22d67b8179c78b9717b2b364ff7cd6"
TARGET_BINARY_SHA256 = "ed24b57b29f679123c3a52dbb04fad9b979092564abc4a35905bdb2abfaa0c8e"

ROUTES = contract.ROUTES
RAW_NAMES = contract.RAW_NAMES
BASE_NAME = "base.obs"
ROUTE_INPUTS = p130.ROUTE_INPUTS


class Phase132ExecutionError(ValueError):
    """Authorization or one-shot structural execution failure."""


def fail(message: str) -> Phase132ExecutionError:
    return Phase132ExecutionError(message)


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
    """Hash static artifacts only; this guard runs before any payload read."""
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


def verify_authorization() -> dict[str, Any]:
    """Verify the complete Phase132 pin chain without opening route payloads."""
    contract.verify_pre_raw_static()
    auth = read_json(AUTHORIZATION, "Phase132 independent raw authorization")
    for key, expected in {
        "schema_version": "smartphone-r5-phase132-typed-canonical-preflight-structural-authorization.v1",
        "phase": 132, "execution_label": "Luna Max",
        "status": "independent-one-shot-inventory-first-structural-raw-authorized",
    }.items():
        assert_equal(auth.get(key), expected, f"authorization/{key}")
    pins = auth.get("pins")
    if not isinstance(pins, dict):
        raise fail("authorization/pins missing")
    expected_pins = {
        "root_cause_freeze_commit": "3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06",
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "contract_audit_commit": AUDIT_COMMIT,
        "contract_freeze_commit": FREEZE_COMMIT,
        "contract_code_commit": CONTRACT_CODE_COMMIT,
        "manifest_commit": MANIFEST_COMMIT,
        "pre_raw_accounting_commit": PRE_RAW_COMMIT,
        "audit_sha256": AUDIT_SHA256, "freeze_sha256": FREEZE_SHA256,
        "manifest_sha256": MANIFEST_SHA256, "pre_raw_sha256": PRE_RAW_SHA256,
        "target_binary_path": relative(contract.BINARY),
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    for key, expected in expected_pins.items():
        assert_equal(pins.get(key), expected, f"authorization/pins/{key}")
    assert_equal(pins.get("authorized_runner_sha256"),
                 static_sha(AUTHORIZED_RUNNER, "Phase132 authorized runner"),
                 "authorization/pins/authorized_runner_sha256")
    runner_commit = pins.get("authorized_runner_commit")
    if (not isinstance(runner_commit, str) or len(runner_commit) != 40 or
            any(char not in "0123456789abcdef" for char in runner_commit)):
        raise fail("authorization/pins/authorized_runner_commit must be full lowercase SHA")
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
        "route_order": ["MTV-A", "LAX-T"], "runs_per_route": 1,
        "controls": 0, "reruns": 0, "fallbacks": 0,
    }.items():
        assert_equal(scope_meta.get(key), expected, f"authorization_scope/{key}")
    selectors = auth.get("selectors")
    if not isinstance(selectors, dict):
        raise fail("authorization/selectors missing")
    for key, expected in {
        "phase126": 1, "phase127": 1, "phase128": 1, "phase129": 1,
        "phase130": 1, "phase131": 1, "phase118": 1,
        "phase117": 0, "phase120": 0, "additional_frequency": 0,
    }.items():
        assert_equal(selectors.get(key), expected, f"authorization/selectors/{key}")
    routes = auth.get("routes")
    if not isinstance(routes, list) or [item.get("dataset_id") for item in routes] != list(ROUTES):
        raise fail("authorization route order/count changed")
    for record in routes:
        route = record.get("dataset_id")
        if route not in ROUTE_INPUTS or record.get("runs") != 1:
            raise fail(f"authorization/{route}: route metadata changed")
        for name in RAW_NAMES:
            actual = record.get("raw_inputs", {}).get(name)
            expected = ROUTE_INPUTS[route][name]
            if not isinstance(actual, dict):
                raise fail(f"authorization/{route}/{name} missing")
            for key in ("path", "bytes", "sha256"):
                assert_equal(actual.get(key), expected[key],
                             f"authorization/{route}/{name}/{key}")
            assert_equal(actual.get("read_before_authorization"), False,
                         f"authorization/{route}/{name}/read_before_authorization")
            assert_equal(actual.get("copy_or_transform"), False,
                         f"authorization/{route}/{name}/copy_or_transform")
        base = record.get("base_input")
        expected = ROUTE_INPUTS[route][BASE_NAME]
        if not isinstance(base, dict):
            raise fail(f"authorization/{route}/base_input missing")
        for key in ("path", "bytes", "sha256"):
            assert_equal(base.get(key), expected[key],
                         f"authorization/{route}/base/{key}")
        for key, value in {
            "read_before_authorization": False,
            "hash_read_before_authorization": False,
            "copy_or_transform": False,
            "coordinate_source": "raw base RINEX header APPROX POSITION XYZ only",
        }.items():
            assert_equal(base.get(key), value, f"authorization/{route}/base/{key}")
    return auth


def _phase132_evidence(marker: dict[str, Any], *, native: bool) -> dict[str, Any]:
    """Expose typed/native reachability while retaining Phase131 evidence."""
    return {
        "enabled": True,
        "selector": contract.PHASE131_SELECTOR,
        "python_typed_canonical_preflight_call_count": int(
            marker.get("python_preflight_call_count", 0) or 0),
        "python_typed_canonical_preflight_executed": marker.get(
            "python_preflight_executed") is True,
        "phase130_literal_preflight_call_count": 0,
        "phase130_literal_preflight_executed": False,
        "native_selector_forwarded": native and marker.get(
            "native_selector_forwarded") is True,
        "native_command_constructed": native and marker.get(
            "native_command_constructed") is True,
        "native_binary_invocation_attempted": native and marker.get(
            "native_binary_invocation_attempted") is True,
        "native_resolver_executed": native and marker.get(
            "native_resolver_executed") is True,
        "native_resolver_call_count": int(
            marker.get("native_resolver_call_count", 0) or 0),
        "native_resolver_evidence_source": marker.get(
            "native_resolver_evidence_source", "not-launched"),
        "native_resolver_call_count_semantics": (
            "native summary canonical rows plus canonical rejected rows; "
            "zero means no canonicalizer row reached"),
        "old_phase130_counters_comparison_only": True,
        "old_literal_join_used": False,
    }


def _phase132ize_result(result: dict[str, Any]) -> dict[str, Any]:
    """Rename only structural metadata namespaces; never inspect solution bytes."""
    route_records = result.get("routes", {})
    evidence_routes = 0
    resolver_routes = 0
    resolver_calls = 0
    for route, record in route_records.items():
        inventory = record.get("inventory", {})
        marker = inventory.get("phase131", {})
        native_reached = bool(record.get("summary_present") and
                              record.get("structural_telemetry"))
        phase = _phase132_evidence(marker, native=native_reached)
        inventory["phase132"] = phase
        if phase["python_typed_canonical_preflight_executed"]:
            evidence_routes += 1
        if phase["native_resolver_executed"]:
            resolver_routes += 1
            resolver_calls += phase["native_resolver_call_count"]
        record["inventory"] = inventory
        telemetry = record.get("structural_telemetry")
        if isinstance(telemetry, dict):
            telemetry["phase132"] = phase
            telemetry["execution_evidence"]["phase130_literal_preflight_call_count"] = 0
            telemetry["execution_evidence"]["phase132_typed_preflight_call_count"] = phase[
                "python_typed_canonical_preflight_call_count"]
            telemetry["execution_evidence"]["old_literal_join_used"] = False
            record["structural_telemetry"] = telemetry
    result["schema_version"] = "smartphone-r5-phase132-typed-canonical-preflight-structural-result.v1"
    result["phase"] = 132
    result["status"] = ("go-phase132-typed-canonical-preflight-structural"
                         if all(all(record.get("gates", {}).values())
                                for record in route_records.values())
                         else "no-go-phase132-typed-canonical-preflight-structural")
    result["decision"] = ("All inventory and structural gates passed; truth/accuracy and solution release remain unauthorized."
                           if result["status"].startswith("go-") else
                           "Inventory or structural gate failed closed; no rerun, fallback, truth, accuracy, or solution release is permitted.")
    result["authorization"] = {
        "path": relative(AUTHORIZATION), "status": "independent-one-shot-inventory-first-structural-raw-authorized",
        "independent": True,
    }
    result["contract"] = {
        "root_cause_freeze_commit": "3b3c785f5a641bc3f2f49ff4e704f80b41a7ea06",
        "audit_commit": AUDIT_COMMIT, "freeze_commit": FREEZE_COMMIT,
        "implementation_commit": IMPLEMENTATION_COMMIT,
        "contract_code_commit": CONTRACT_CODE_COMMIT,
        "manifest_commit": MANIFEST_COMMIT, "pre_raw_commit": PRE_RAW_COMMIT,
        "manifest_sha256": MANIFEST_SHA256,
        "target_binary_sha256": TARGET_BINARY_SHA256,
        "authorized_runner_path": relative(AUTHORIZED_RUNNER),
    }
    result["candidate"] = {
        "id": contract.CANDIDATE_ID, "phase126": True, "phase127": True,
        "phase128": True, "phase129": True, "phase130": True,
        "phase131": True, "phase118_huber": True,
        "phase117_dynamic_sigma": False, "phase120_atmosphere": False,
        "additional_frequency_bands": False, "fixed_tdcp_sigma": 0.03,
        "official_huber_k": 0.5, "main_solver": "MULTIFRONTAL_QR",
        "solution_opaque": True,
    }
    result["phase132_execution_evidence"] = {
        "selector": contract.PHASE131_SELECTOR,
        "typed_preflight_executed_routes": evidence_routes,
        "native_command_constructed_routes": sum(
            1 for record in route_records.values()
            if record.get("inventory", {}).get("phase132", {}).get(
                "native_command_constructed") is True),
        "native_selector_forwarded_routes": sum(
            1 for record in route_records.values()
            if record.get("inventory", {}).get("phase132", {}).get(
                "native_selector_forwarded") is True),
        "native_resolver_executed_routes": resolver_routes,
        "native_resolver_call_count": resolver_calls,
        "phase130_literal_preflight_call_count": 0,
        "phase130_counters_are_comparison_only": True,
        "call_count_semantics": "typed Python preflight count is per-route inventory; native resolver count is canonical rows plus rejected rows",
    }
    result["matrix"]["phase"] = 132
    result["matrix"]["old_phase130_literal_preflight_calls"] = 0
    result["truth_free"] = True
    result["accuracy_scored"] = False
    result["solution_output_published"] = False
    result["promotion_authorized"] = False
    result["stop_before_truth_accuracy_submission"] = True
    return result


def result_markdown(result: dict[str, Any]) -> str:
    lines = [
        "# Phase132 typed canonical preflight structural raw result", "",
        f"- Status: `{result['status']}`",
        "- Matrix: MTV-A then LAX-T, exactly one inventory pass and at most one native solver attempt per route.",
        "- Typed preflight key: `(GNSSSystem, PRN, physical-frequency-family[, certified GLONASS FCN])`; literal text is provenance only.",
        "- Phase130 literal preflight calls are sealed as `0`; Phase130 counters are comparison-only.",
        "- Solution rows were not opened or interpreted; only opaque hashes and row counts were sealed.",
        "- Truth, MAT, PDC, precomputed coordinates, accuracy, and Kaggle remain unauthorized.", "",
        "| Route | Inventory | Solver | Return | Typed calls | Old literal calls | Native resolver calls | Main accepted | Main cost | GO | Failure |",
        "|---|---|---|---:|---:|---:|---:|---:|---|---|---|",
    ]
    for route in ROUTES:
        record = result.get("routes", {}).get(route, {})
        inventory = record.get("inventory", {})
        phase = inventory.get("phase132", {})
        main = record.get("structural_telemetry", {}).get("main", {})
        gates = record.get("gates", {})
        failure = record.get("failure") or inventory.get("failure") or record.get("summary_error") or ""
        lines.append(
            f"| `{route}` | `{inventory.get('ok')}` | `{record.get('solver_launched', False)}` | "
            f"`{record.get('return_code')}` | `{phase.get('python_typed_canonical_preflight_call_count', 0)}` | `0` | "
            f"`{phase.get('native_resolver_call_count', 0)}` | `{main.get('iterations', 0)}` | "
            f"`{main.get('initial_cost')}->{main.get('final_cost')}` | "
            f"`{all(gates.values()) if gates else False}` | `{failure}` |")
    lines.extend(["", "Inventory failure is fail-closed and prevents native launch for that route.", ""])
    return "\n".join(lines)


def execute_matrix() -> dict[str, Any]:
    """Run the already-qualified implementation after this adapter's auth gate."""
    verify_authorization()
    if RESULT_JSON.exists() or RESULT_MD.exists() or OUTPUT_ROOT.exists():
        raise fail("refusing to overwrite an existing Phase132 result/output")
    # Keep the native implementation and its one-shot raw accounting, but
    # point all mutable artifacts at the Phase132 namespace.  Its contract
    # module remains Phase131 so the graph/factor implementation is untouched.
    implementation.AUTHORIZATION = AUTHORIZATION
    implementation.PRE_RAW = PRE_RAW
    implementation.MANIFEST = MANIFEST
    implementation.OUTPUT_ROOT = OUTPUT_ROOT
    implementation.RESULT_JSON = RESULT_JSON
    implementation.RESULT_MD = RESULT_MD
    implementation.AUTHORIZED_RUNNER = AUTHORIZED_RUNNER
    implementation.AUDIT_COMMIT = AUDIT_COMMIT
    implementation.FREEZE_COMMIT = FREEZE_COMMIT
    implementation.IMPLEMENTATION_COMMIT = IMPLEMENTATION_COMMIT
    implementation.MANIFEST_COMMIT = MANIFEST_COMMIT
    implementation.PRE_RAW_COMMIT = PRE_RAW_COMMIT
    implementation.AUDIT_SHA256 = AUDIT_SHA256
    implementation.FREEZE_SHA256 = FREEZE_SHA256
    implementation.MANIFEST_SHA256 = MANIFEST_SHA256
    implementation.TARGET_BINARY_SHA256 = TARGET_BINARY_SHA256
    def authorized_again() -> dict[str, Any]:
        return verify_authorization()

    implementation.verify_authorization = authorized_again
    result = implementation.execute_matrix()
    result = _phase132ize_result(result)
    for route, record in result.get("routes", {}).items():
        inventory = record.get("inventory", {})
        route_dir = OUTPUT_ROOT / route.replace("/", "__")
        inventory_path = route_dir / "inventory.json"
        if inventory_path.is_file():
            implementation.atomic_json(inventory_path, inventory)
        summary_path = route_dir / "structural_summary.json"
        telemetry = record.get("structural_telemetry")
        if summary_path.is_file() and isinstance(telemetry, dict):
            summary = read_json(summary_path, f"Phase132 structural summary {route}")
            summary["schema_version"] = "smartphone-r5-phase132-typed-canonical-preflight-structural-summary.v1"
            summary["phase"] = 132
            summary["phase132"] = telemetry.get("phase132", {})
            implementation.atomic_json(summary_path, summary)
    implementation.atomic_json(RESULT_JSON, result)
    RESULT_MD.write_text(result_markdown(result), encoding="utf-8")
    return result


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-authorization", action="store_true",
                        help="verify all static pins without opening route payloads")
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
                    "phase132", {}).get("python_typed_canonical_preflight_call_count", 0),
                "old_literal_preflight_calls": 0,
                "native_resolver_calls": result["routes"][route]["inventory"].get(
                    "phase132", {}).get("native_resolver_call_count", 0),
                "failure": result["routes"][route].get("failure") or
                           result["routes"][route]["inventory"].get("failure"),
            } for route in ROUTES},
            "read_accounting": result["read_accounting"],
        }, indent=2, sort_keys=True))
        return 0 if result["status"].startswith("go-") else 3
    except (Phase132ExecutionError, implementation.Phase131ExecutionError, OSError) as exc:
        print(f"phase132 authorized runner: fail-closed: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
