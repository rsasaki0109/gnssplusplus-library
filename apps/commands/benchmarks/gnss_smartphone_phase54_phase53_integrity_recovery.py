#!/usr/bin/env python3
"""Recover the Phase53 raw-input-integrity presentation gate without rereads.

Phase54 is intentionally a scorer-only operation.  It reads the immutable
Phase53 output manifest, result, routes, and empty event table once each,
checks their hashes, and recomputes only presentation/integrity predicates.
No Phase53 raw input, truth, navigation, solver output, or prior metric
payload is opened.  The physical Phase53 no-go evidence is copied and checked
for immutability; it is never reinterpreted as a promotion path.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase54_phase53_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "82d4235b4b7cfc8d0536ecc615d92be535c3bf8b6ba4b1369e40994675e0ed00"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase54_phase53_integrity_recovery_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase54-phase53-integrity-recovery-v1"

SCHEMA = "smartphone-r5-phase54-phase53-integrity-recovery.v1"
FREEZE_SCHEMA = "smartphone-r5-phase54-phase53-integrity-recovery-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase54-phase53-integrity-recovery-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)


class Phase54Error(ValueError):
    """Raised when the immutable Phase54 recovery contract is violated."""


def _fail(message: str) -> Phase54Error:
    return Phase54Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "precomputed",
        "device_wls", "svposition", "svelevation", "archive", "kaggle", "token",
        "device_gnss.csv",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase54 input/output path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {exc}") from exc
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be a JSON object")
    return value, digest


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes(value)
    _atomic_write(path, payload)
    return payload


def _median(values: Iterable[float]) -> float:
    data = [float(value) for value in values]
    return float(statistics.median(data)) if data else 0.0


def _mad(values: Iterable[float]) -> float:
    data = [float(value) for value in values]
    return _median(abs(value - _median(data)) for value in data) if data else 0.0


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase54 freeze", FREEZE_SHA256 or None)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase54 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase54-scorer":
        raise _fail("Phase54 freeze schema/status mismatch")
    pins = freeze.get("phase53_pins")
    if not isinstance(pins, dict):
        raise _fail("Phase54 Phase53 artifact pins missing")
    required = ("result", "routes", "events", "output_manifest")
    for name in required:
        item = pins.get(name, {})
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("bytes", 0)) <= 0:
            raise _fail(f"Phase54 Phase53 {name} pin missing")
    if freeze.get("integrity_contract", {}).get("corrected_raw_input_integrity_definition") != (
        "exact frozen SHA/byte checks AND all core relation fields finite AND nonmonotonic epoch count equals zero AND duplicate epoch-key count equals zero"
    ):
        raise _fail("Phase54 corrected integrity definition changed")
    unsupported = freeze.get("integrity_contract", {}).get("unsupported_signal_rows", {})
    if unsupported.get("gate_role") != "informational_only_excluded_from_raw_input_integrity":
        raise _fail("Phase54 unsupported-signal policy changed")
    policy = freeze.get("phase54_input_policy", {})
    expected_policy = {
        "single_process": True,
        "phase53_output_manifest_reads": 1,
        "phase53_result_reads": 1,
        "phase53_routes_reads": 1,
        "phase53_events_reads": 1,
        "phase53_docs_or_record_reads": 0,
        "raw_device_gnss_reads": 0,
        "truth_reads": 0,
        "navigation_reads": 0,
        "solver_invocations": 0,
        "trajectory_invocations": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "kaggle_token_access": 0,
        "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0,
        "SvPosition_or_SvElevation": 0,
        "output_mutation": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase54 input policy changed")
    assertions = freeze.get("pre_read_assertions", {})
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase54 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase54 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase54 manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase54-scorer":
        raise _fail("Phase54 manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase54 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "integrity-recovery" or evaluator.get("native_solver_invoked") is not False or evaluator.get("single_process") is not True:
        raise _fail("Phase54 manifest permits solver/multi-process execution")
    if evaluator.get("phase53_raw_reopen") is not False or evaluator.get("phase53_output_mutation") is not False:
        raise _fail("Phase54 manifest permits Phase53 raw/output mutation")
    if evaluator.get("phase53_artifact_reads_per_file") != 1:
        raise _fail("Phase54 artifact read accounting changed")
    forbidden = manifest.get("forbidden", [])
    required_forbidden = ("raw device input", "ground truth", "navigation", "solver", "prior metric payload", "output mutation")
    if not isinstance(forbidden, list) or not all(token in forbidden for token in required_forbidden):
        raise _fail("Phase54 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase54 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase54 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _phase53_bundle(freeze: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any], dict[str, Any]]:
    pins = freeze["phase53_pins"]
    manifest_path = ROOT / pins["output_manifest"]["path"]
    output_manifest, output_digest = _load_json_once(
        manifest_path, "Phase53 output manifest", pins["output_manifest"]["sha256"]
    )
    if output_digest != pins["output_manifest"]["sha256"]:
        raise _fail("Phase53 output manifest digest changed")
    artifacts = output_manifest.get("artifacts", {})
    for name in ("result", "routes", "events"):
        if artifacts.get(name, {}).get("sha256") != pins[name]["sha256"] or int(artifacts.get(name, {}).get("bytes", 0)) != int(pins[name]["bytes"]):
            raise _fail(f"Phase53 output manifest {name} pin mismatch")
    result, result_digest = _load_json_once(
        ROOT / pins["result"]["path"], "Phase53 result", pins["result"]["sha256"]
    )
    routes, routes_digest = _load_json_once(
        ROOT / pins["routes"]["path"], "Phase53 routes", pins["routes"]["sha256"]
    )
    events, events_digest = _load_json_once(
        ROOT / pins["events"]["path"], "Phase53 events", pins["events"]["sha256"]
    )
    if result_digest != pins["result"]["sha256"] or routes_digest != pins["routes"]["sha256"] or events_digest != pins["events"]["sha256"]:
        raise _fail("Phase53 artifact digest changed")
    if result.get("phase") != 53 or routes.get("phase") != 53 or events.get("phase") != 53:
        raise _fail("Phase53 artifact phase mismatch")
    if tuple(routes.get("route_order", [])) != ROUTES:
        raise _fail("Phase53 route artifact order changed")
    if set(routes.get("routes", {})) != set(ROUTES):
        raise _fail("Phase53 route artifact map changed")
    return output_manifest, result, routes, events, {
        "output_manifest": output_digest,
        "result": result_digest,
        "routes": routes_digest,
        "events": events_digest,
    }


def _recompute_integrity(result: dict[str, Any], routes_artifact: dict[str, Any], events: dict[str, Any], output_manifest: dict[str, Any]) -> dict[str, Any]:
    routes = routes_artifact["routes"]
    route_checks: dict[str, dict[str, bool]] = {}
    for route in ROUTES:
        report = routes[route]
        rows = report.get("rows", {})
        groups = report.get("groups", {})
        pair_reasons = report.get("pair_reasons", {})
        signal_groups = groups.get("signal_frequency", {})
        satellite_groups = groups.get("satellite", {})
        state_groups = groups.get("state", {})
        route_checks[route] = {
            "pair_reason_counts_sum": sum(int(value) for value in pair_reasons.values()) == int(rows.get("pair_count", -1)),
            "state_group_counts_sum": sum(int(value.get("count", -1)) for value in state_groups.values()) == int(rows.get("pair_count", -1)),
            "signal_frequency_group_counts_sum": sum(int(value.get("count", -1)) for value in signal_groups.values()) == int(rows.get("ordinary_pair_count", -1)),
            "satellite_group_counts_sum": sum(int(value.get("count", -1)) for value in satellite_groups.values()) == int(rows.get("ordinary_pair_count", -1)),
            "header_field_presence": isinstance(report.get("headers", {}).get("columns"), list) and isinstance(report.get("headers", {}).get("optional_field_presence"), dict),
            "all_core_fields_finite": bool(report.get("gate_observations", {}).get("all_core_finite")) and all(
                _finite(report.get("residuals", {}).get(name, {}).get(metric))
                for name, metric in (
                    ("frequency_aware_centered_m", "median"),
                    ("frequency_aware_abs_m", "p95_abs"),
                    ("constant_frequency_control_abs_m", "p95_abs"),
                    ("frequency_leakage_abs_m", "p95_abs"),
                )
            ),
            "nonmonotonic_epochs_zero": int(rows.get("nonmonotonic_epoch_key_count", -1)) == 0,
            "duplicate_epoch_keys_zero": int(rows.get("repeated_epoch_key_count", -1)) == 0,
        }
    raw_integrity = {
        route: {
            "exact_sha256_and_bytes": True,
            "all_core_fields_finite": checks["all_core_fields_finite"],
            "nonmonotonic_epochs_zero": checks["nonmonotonic_epochs_zero"],
            "duplicate_epoch_keys_zero": checks["duplicate_epoch_keys_zero"],
            "unsupported_signal_rows_informational": int(routes[route]["rows"].get("unsupported_signal_rows", -1)),
            "passed": bool(
                checks["all_core_fields_finite"]
                and checks["nonmonotonic_epochs_zero"]
                and checks["duplicate_epoch_keys_zero"]
            ),
        }
        for route, checks in route_checks.items()
    }
    route_medians = {
        route: float(routes[route]["residuals"]["frequency_aware_abs_m"]["median"])
        for route in ROUTES
    }
    expected_route_medians = result.get("aggregate", {}).get("route_median_abs_frequency_residual_m", {})
    route_medians_match = (
        isinstance(expected_route_medians, dict)
        and tuple(expected_route_medians) == ROUTES
        and all(_finite(expected_route_medians.get(route)) and float(expected_route_medians[route]) == route_medians[route] for route in ROUTES)
    )
    median_aggregate = _median(route_medians.values())
    mad_aggregate = _mad(route_medians.values())
    expected_aggregate = result.get("aggregate", {})
    aggregate_exact = (
        median_aggregate == float(expected_aggregate.get("route_median_abs_frequency_residual_aggregate_m", math.nan))
        and mad_aggregate == float(expected_aggregate.get("route_median_abs_frequency_residual_mad_m", math.nan))
    )
    events_count_exact = int(events.get("count", -1)) == int(result.get("events", {}).get("count", -2))
    output_artifacts = output_manifest.get("artifacts", {})
    output_pins_exact = all(
        name in output_artifacts and len(str(output_artifacts[name].get("sha256", ""))) == 64
        for name in ("result", "routes", "events")
    )
    presentation = {
        "route_checks": route_checks,
        "pair_reason_counts_sum_all": all(item["pair_reason_counts_sum"] for item in route_checks.values()),
        "state_group_counts_sum_all": all(item["state_group_counts_sum"] for item in route_checks.values()),
        "signal_frequency_group_counts_sum_all": all(item["signal_frequency_group_counts_sum"] for item in route_checks.values()),
        "satellite_group_counts_sum_all": all(item["satellite_group_counts_sum"] for item in route_checks.values()),
        "header_field_presence_route_map_exact": all(item["header_field_presence"] for item in route_checks.values()),
        "four_route_medians_retained": tuple(route_medians) == ROUTES and route_medians_match,
        "aggregate_recomputed_exact": aggregate_exact,
        "loo_route_count_exact": len(result.get("loo", {}).get("folds", [])) == len(ROUTES) and tuple(item.get("omitted_route") for item in result["loo"]["folds"]) == ROUTES,
        "event_count_exact": events_count_exact,
        "output_manifest_artifact_map_exact": output_pins_exact,
    }
    presentation["all_passed"] = all(value for key, value in presentation.items() if key not in {"route_checks"})
    raw_summary = {
        "route_checks": raw_integrity,
        "all_passed": all(item["passed"] for item in raw_integrity.values()),
        "unsupported_signal_rows_role": "informational_only_excluded_from_gate",
    }
    return {
        "raw_input_integrity": raw_summary,
        "presentation_integrity": presentation,
        "route_medians_recomputed_m": route_medians,
        "recomputed_route_median_aggregate_m": median_aggregate,
        "recomputed_route_median_mad_m": mad_aggregate,
    }


def _physical_no_go(result: dict[str, Any], routes_artifact: dict[str, Any], recomputed: dict[str, Any]) -> dict[str, Any]:
    original = result.get("gates", {}).get("observed", {})
    reports = routes_artifact.get("routes", {})
    antenna_absent = all(not bool(reports[route].get("antenna_phase_bias", {}).get("available")) for route in ROUTES)
    one_group = all(int(reports[route].get("rows", {}).get("signal_frequency_group_count", 0)) == 1 for route in ROUTES)
    leakage = [float(reports[route]["gate_observations"]["frequency_leakage_p95_abs_m"]) for route in ROUTES]
    excess = [float(reports[route]["gate_observations"]["frequency_vs_control_p95_excess_m"]) for route in ROUTES]
    spearman = [float(reports[route]["gate_observations"]["spearman"]) for route in ROUTES]
    physical = {
        "phase53_status_unchanged": result.get("status") == "no-go-carrier-frequency-antenna-phase-bias-not-identifiable",
        "phase53_all_gates_passed_false": result.get("gates", {}).get("all_passed") is False,
        "antenna_source_absent_all_routes": antenna_absent,
        "one_signal_frequency_group_all_routes": one_group,
        "frequency_leakage_p95_below_0_03_m_all_routes": all(value < 0.03 for value in leakage),
        "frequency_vs_control_excess_below_0_02_m_all_routes": all(value < 0.02 for value in excess),
        "routewise_spearman_zero": all(value == 0.0 for value in spearman),
        "correction_authorized_false": result.get("decision", {}).get("correction_authorized") is False,
        "next_factor_unchanged": result.get("decision", {}).get("next_single_raw_physical_factor") == "raw Android per-satellite accumulated-delta-range uncertainty (AccumulatedDeltaRangeUncertaintyMeters)",
    }
    physical["all_passed"] = all(physical.values())
    return {
        "checks": physical,
        "immutable_no_go": bool(physical["all_passed"]),
        "leakage_p95_abs_m": leakage,
        "frequency_vs_control_p95_excess_m": excess,
        "spearman": spearman,
        "source_gate_values": {
            "phase53_antenna_source": original.get("antenna_source"),
            "phase53_frequency_groups": original.get("frequency_groups"),
            "phase53_frequency_materiality": original.get("frequency_materiality"),
            "phase53_relation_identifiability": original.get("relation_identifiability"),
        },
    }


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase54 output already exists: {output_root}")
    output_manifest, result, routes_artifact, events, digests = _phase53_bundle(freeze)
    recomputed = _recompute_integrity(result, routes_artifact, events, output_manifest)
    physical = _physical_no_go(result, routes_artifact, recomputed)
    if not physical["immutable_no_go"]:
        raise _fail("Phase53 physical no-go evidence changed or is not reproducible")
    if not recomputed["raw_input_integrity"]["all_passed"]:
        raise _fail("corrected raw-input-integrity recovery failed")
    if not recomputed["presentation_integrity"]["all_passed"]:
        raise _fail("Phase53 presentation-integrity recomputation failed")
    reads = {
        "single_process": True,
        "phase53_output_manifest_reads": 1,
        "phase53_result_reads": 1,
        "phase53_routes_reads": 1,
        "phase53_events_reads": 1,
        "phase53_docs_or_record_reads": 0,
        "phase53_raw_device_gnss_rereads": 0,
        "raw_device_gnss_reads": 0,
        "raw_device_imu_reads": 0,
        "truth_reads": 0,
        "past_truth_payload_reads": 0,
        "phase52_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "correction_implementations": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "kaggle_token_access": 0,
        "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0,
        "SvPosition_or_SvElevation": 0,
        "phase53_output_mutations": 0,
    }
    corrected_gates = {
        "raw_input_integrity": True,
        "presentation_integrity": True,
        "physical_phase53_no_go_preserved": physical["immutable_no_go"],
        "phase53_unsupported_signal_rows_informational_only": True,
        "no_accuracy_scoring": True,
    }
    output_result = {
        "schema_version": SCHEMA,
        "phase": 54,
        "execution_label": "Luna Max",
        "status": "phase53-integrity-recovered-physical-no-go",
        "operation": "scorer-only immutable Phase53 integrity recovery",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "phase53_artifact_pins": freeze["phase53_pins"],
        "phase53_artifact_digests_verified": digests,
        "phase53_original_gate": {
            "status": result.get("status"),
            "raw_input_integrity": result.get("gates", {}).get("observed", {}).get("raw_input_integrity"),
            "presentation_integrity": result.get("gates", {}).get("observed", {}).get("presentation_integrity"),
        },
        "corrected_gates": corrected_gates,
        "recomputed": recomputed,
        "physical_no_go": physical,
        "read_accounting": reads,
        "decision": {
            "native_correction_authorized": False,
            "solver_or_algorithm_change": False,
            "truth_or_metric_scoring": False,
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "next_single_raw_physical_factor": "raw Android per-satellite accumulated-delta-range uncertainty (AccumulatedDeltaRangeUncertaintyMeters)",
            "zero_point_782": "not evaluated without truth",
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    result_path = output_root / "phase54_phase53_integrity_recovery.json"
    result_payload = _atomic_json(result_path, output_result)
    artifact_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 54,
        "freeze_sha256": FREEZE_SHA256,
        "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "phase53_input_digests": digests,
        "read_accounting": reads,
        "artifacts": {
            "result": {"path": _relative(result_path), "bytes": len(result_payload), "sha256": _sha256_bytes(result_payload)},
        },
    }
    _atomic_json(output_root / "phase54_phase53_integrity_recovery.manifest.json", artifact_manifest)
    return output_result


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify the closed freeze and manifest without Phase53 artifact reads")
    parser.add_argument("--recover", action="store_true", help="run the one-shot immutable-artifact scorer")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    if args.verify_freeze == args.recover:
        parser.error("choose exactly one of --verify-freeze or --recover")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase54 freeze/evaluator manifest: verified without Phase53 artifact reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({
            "status": result["status"],
            "corrected_raw_input_integrity": result["corrected_gates"]["raw_input_integrity"],
            "physical_no_go_preserved": result["corrected_gates"]["physical_phase53_no_go_preserved"],
            "phase53_artifact_reads": sum(result["read_accounting"][key] for key in ("phase53_output_manifest_reads", "phase53_result_reads", "phase53_routes_reads", "phase53_events_reads")),
            "raw_rereads": result["read_accounting"]["phase53_raw_device_gnss_rereads"],
        }, sort_keys=True))
        return 0
    except Phase54Error as exc:
        print(f"phase54 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
