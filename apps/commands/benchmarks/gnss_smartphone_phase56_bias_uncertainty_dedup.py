#!/usr/bin/env python3
"""Verify Phase56's no-raw BiasUncertaintyNanos deduplication evidence.

This is deliberately a sealed-artifact/source audit.  It never opens a new
device_gnss.csv, truth, navigation, solver output, coordinate, or IMU input.
The Phase46 raw audit remains immutable; this command only checks its pinned
records and output artifacts, then statically verifies the current adapter and
FGO uncertainty plumbing before publishing a Phase56 evidence map.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any, Iterable


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase56_pixel5_bias_uncertainty_dedup_freeze_v1.json"
FREEZE_SHA256 = "b6242ed176fbe124b523d635575572f12d25494082f5c57c837d3c3c5fe35567"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase56_bias_uncertainty_dedup_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase56-pixel5-bias-uncertainty-dedup-v1"

SCHEMA = "smartphone-r5-phase56-pixel5-bias-uncertainty-dedup.v1"
FREEZE_SCHEMA = "smartphone-r5-phase56-pixel5-bias-uncertainty-dedup-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase56-pixel5-bias-uncertainty-dedup-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)


class Phase56Error(ValueError):
    """Raised when a sealed Phase56 evidence contract is violated."""


def _fail(message: str) -> Phase56Error:
    return Phase56Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "kaggle",
        "token", "archive", "device_wls", "precomputed", "svposition",
        "svelevation", "device_imu",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase56 path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(path: Path, label: str, expected_sha256: str | None = None, expected_bytes: int | None = None) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {exc}") from exc
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise _fail(f"{label} byte count mismatch: {len(payload)} != {expected_bytes}")
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None, expected_bytes: int | None = None) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256, expected_bytes)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be an object")
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


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase56 freeze", FREEZE_SHA256 or None)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase56 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase56-new-raw-read":
        raise _fail("Phase56 freeze schema/status mismatch")
    if freeze.get("scope", {}).get("new_raw_reads") != 0:
        raise _fail("Phase56 freeze permits a new raw read")
    if freeze.get("scope", {}).get("truth_or_metric_payload_inputs") != 0:
        raise _fail("Phase56 freeze permits truth/metric input")
    pins = freeze.get("authority_pins", {})
    required_pins = (
        "phase46_freeze", "phase46_evaluator_manifest", "phase46_result_record",
        "phase46_output_manifest", "phase46_output_result", "phase46_output_routes",
        "phase46_output_events", "phase55_policy_result",
    )
    if any(not isinstance(pins.get(name), dict) for name in required_pins):
        raise _fail("Phase56 sealed evidence pins incomplete")
    evidence = freeze.get("phase46_evidence_map", {})
    if tuple(evidence.get("cohort_route_order", [])) != ROUTES:
        raise _fail("Phase56 route order changed")
    reads = evidence.get("read_accounting_sealed", {})
    if reads.get("single_evaluator_process") is not True or reads.get("raw_device_gnss_reads_per_route") != 1 or reads.get("truth_reads") != 0:
        raise _fail("Phase46 sealed read accounting mismatch")
    bias = evidence.get("bias_uncertainty_source", {})
    if bias.get("field") != "BiasUncertaintyNanos" or bias.get("units") != "nanoseconds" or bias.get("owner") != "Android GnssClock receiver clock":
        raise _fail("BiasUncertaintyNanos source semantics changed")
    common = evidence.get("phase46_common_mode_evidence", {})
    for key, expected in {
        "status": "no-go-clock-correction-common-mode-only",
        "same_epoch_receiver_tow_spread_max_m_all_routes": 0.0,
        "same_epoch_clock_field_inconsistency_count_all_routes": 0,
        "same_epoch_signal_group_median_spread_max_ms_all_routes": 0.0,
        "geometry_changing_noncommon_tow_effect_max_m": 0.0,
    }.items():
        if common.get(key) != expected:
            raise _fail(f"Phase46 common-mode evidence changed: {key}")
    next_factor = freeze.get("next_nonduplicate_factor", {})
    if next_factor.get("field") != "PseudorangeRateUncertaintyMetersPerSecond":
        raise _fail("Phase56 next factor changed")
    source_contracts = freeze.get("source_contracts", {})
    if not isinstance(source_contracts, dict) or len(source_contracts) < 6:
        raise _fail("Phase56 source contract pins incomplete")
    assertions = freeze.get("pre_read_assertions", {})
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase56 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase56 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase56 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evidence-map-evaluator-sealed-before-phase56-run":
        raise _fail("Phase56 evaluator manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase56 evaluator manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("operation") != "sealed-artifact-and-source-evidence-map" or evaluator.get("new_raw_reads") != 0 or evaluator.get("truth_reads") != 0 or evaluator.get("single_process") is not True:
        raise _fail("Phase56 evaluator manifest permits forbidden inputs")
    forbidden = manifest.get("forbidden", [])
    required_forbidden = ("new raw device_gnss.csv", "truth", "navigation", "solver", "coordinates", "IMU", "MAT", "validation", "holdout", "Kaggle", "token", "prior metric payload")
    if not isinstance(forbidden, list) or not all(item in forbidden for item in required_forbidden):
        raise _fail("Phase56 evaluator forbidden policy incomplete")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase56 {name} pin missing")
        if _sha256_bytes(pin_path.read_bytes()) != pin["sha256"]:
            raise _fail(f"Phase56 {name} pin hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _source_evidence(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["source_contracts"].items():
        path = ROOT / str(pin["path"])
        payload, digest = _read_bytes_once(path, f"Phase56 source {name}", str(pin["sha256"]))
        try:
            contents[name] = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise _fail(f"Phase56 source is not UTF-8: {name}") from exc
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    adapter_header = contents["android_raw_gnss_header"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    fgo_config = contents["fgo_config_header"]
    cli = contents["gnss_fgo_cli"]
    doppler = contents["doppler_contract"]
    adapter_lower = adapter.lower()
    observation_lower = observation.lower()
    fgo_lower = fgo.lower()
    return {
        "source_hashes": hashes,
        "adapter": {
            "parses_pseudorange_rate_mps": "pseudorangeratemeterspersecond" in adapter_lower and "pseudorange_rate_mps" in adapter_lower,
            "parses_pseudorange_rate_uncertainty_mps": "pseudorangerateuncertaintymeterspersecond" in adapter_lower,
            "parses_bias_uncertainty_nanos": "biasuncertaintynanos" in adapter_lower and "bias_uncertainty_nanos" in adapter_lower,
            "retains_bias_uncertainty_in_observation": "observation.bias_uncertainty" in adapter_lower,
            "header_declares_rate_uncertainty": "pseudorange_rate_uncertainty" in adapter_header.lower(),
        },
        "observation": {
            "retains_pseudorange_rate_mps": "pseudorange_rate_mps" in observation_lower,
            "retains_pseudorange_rate_uncertainty_mps": "pseudorange_rate_uncertainty" in observation_lower,
            "retains_bias_uncertainty_nanos": "bias_uncertainty" in observation_lower,
        },
        "fgo": {
            "consumes_raw_rate_uncertainty": "pseudorange_rate_uncertainty" in fgo_lower,
            "uses_fixed_undifferenced_doppler_sigma": "undifferenced_doppler_sigma_mps" in fgo_lower,
            "uses_fixed_single_difference_doppler_sigma": "single_difference_doppler_sigma_mps" in fgo_lower,
            "uses_snr_derived_upstream_doppler_sigma": "upstream_doppler_sigma" in fgo_lower,
        },
        "config": {
            "fixed_undifferenced_doppler_sigma_mps": "undifferenced_doppler_sigma_mps = 0.2" in fgo_config,
            "fixed_single_difference_doppler_sigma_mps": "single_difference_doppler_sigma_mps = 0.2" in fgo_config,
        },
        "cli": {
            "raw_rate_uncertainty_option": "pseudorangerateuncertainty" in cli.lower(),
            "fixed_doppler_sigma_option": "undifferenced-doppler-sigma" in cli,
        },
        "doppler_contract": {
            "android_rate_to_doppler_sign": "androidRateToRinexDoppler" in doppler and "-pseudorange_rate_mps" in doppler,
        },
    }


def _verify_artifacts(freeze: dict[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    pins = freeze["authority_pins"]
    reads: dict[str, int] = {}
    sealed: dict[str, Any] = {}
    sealed_json: dict[str, dict[str, Any]] = {}
    # These are sealed non-raw artifacts.  Each is loaded exactly once here.
    for name in ("phase46_freeze", "phase46_evaluator_manifest", "phase46_result_record", "phase46_output_manifest", "phase55_policy_result"):
        pin = pins[name]
        value, digest = _load_json_once(ROOT / pin["path"], f"Phase56 sealed {name}", pin["sha256"])
        reads[name] = 1
        sealed_json[name] = value
        sealed[name] = {"sha256": digest, "schema_version": value.get("schema_version"), "status": value.get("status")}
    for name in ("phase46_output_result", "phase46_output_routes", "phase46_output_events"):
        pin = pins[name]
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase56 sealed {name}", pin["sha256"], int(pin["bytes"]))
        try:
            value = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise _fail(f"invalid Phase56 sealed {name}") from exc
        if not isinstance(value, dict):
            raise _fail(f"Phase56 sealed {name} must be an object")
        reads[name] = 1
        sealed[name] = {"sha256": digest, "schema_version": value.get("schema_version"), "status": value.get("status"), "top_level_keys": sorted(value)}
        if name == "phase46_output_result":
            sealed["phase46_route_evidence"] = value.get("routes")
    # The record was already loaded once above; use only a small policy view
    # from the in-memory object and never use its route metrics as a factor.
    phase46_result = sealed_json["phase46_result_record"]
    sealed["phase46_policy_view"] = {
        "status": phase46_result.get("status"),
        "clock_correction_go": phase46_result.get("decision", {}).get("clock_correction_go"),
        "next_factor": phase46_result.get("decision", {}).get("next_single_raw_physical_factor"),
    }
    # Drop the potentially large route map before returning; all source
    # evidence below is derived only from the pinned policy/evidence fields.
    route_evidence = sealed.pop("phase46_route_evidence")
    if not isinstance(route_evidence, dict) or tuple(route_evidence) != ROUTES:
        raise _fail("Phase46 route evidence map changed")
    counts: dict[str, int] = {}
    medians: list[float] = []
    p95s: list[float] = []
    same_epoch_spreads: list[float] = []
    signal_spreads: list[float] = []
    noncommon: list[float] = []
    for route in ROUTES:
        report = route_evidence[route]
        optional = report.get("optional_fields", {}).get("bias_uncertainty_nanos", {})
        count = int(optional.get("count", 0))
        median = float(optional.get("median", 0.0))
        p95 = float(optional.get("p95_abs", 0.0))
        same = float(report.get("same_epoch_clock_fields", {}).get("per_epoch_spread_ns", {}).get("FullBiasNanos", {}).get("max", 0.0))
        signal = float(report.get("constellation_signal", {}).get("same_epoch_group_median_spread_ms", {}).get("max_abs", 0.0))
        route_noncommon = float(report.get("gate_observations", {}).get("max_uncaptured_noncommon_tow_effect_m", 0.0))
        if count <= 0 or not all(_is_finite(value) for value in (median, p95, same, signal, route_noncommon)):
            raise _fail(f"Phase46 BiasUncertainty evidence is missing/nonfinite: {route}")
        counts[route] = count
        medians.append(median)
        p95s.append(p95)
        same_epoch_spreads.append(same)
        signal_spreads.append(signal)
        noncommon.append(route_noncommon)
    sealed["phase46_bias_uncertainty_route_evidence"] = {
        "count_by_route": counts,
        "median_ns_by_route": medians,
        "p95_abs_ns_by_route": p95s,
        "same_epoch_full_bias_spread_max_ns_by_route": same_epoch_spreads,
        "same_epoch_signal_group_spread_max_ms_by_route": signal_spreads,
        "noncommon_tow_effect_max_m_by_route": noncommon,
        "all_bias_uncertainty_populated": all(count > 0 for count in counts.values()),
        "same_epoch_base_clock_spread_zero": all(value == 0.0 for value in same_epoch_spreads),
        "same_epoch_signal_spread_zero": all(value == 0.0 for value in signal_spreads),
        "noncommon_tow_effect_zero": all(value == 0.0 for value in noncommon),
    }
    return sealed, reads


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase56 output already exists: {output_root}")
    sealed, artifact_reads = _verify_artifacts(freeze)
    source = _source_evidence(freeze)
    route_evidence = sealed["phase46_bias_uncertainty_route_evidence"]
    policy = sealed["phase46_policy_view"]
    expected_next = "PseudorangeRateUncertaintyMetersPerSecond"
    source_checks = {
        "adapter_does_not_parse_rate_uncertainty": source["adapter"]["parses_pseudorange_rate_uncertainty_mps"] is False,
        "observation_does_not_retain_rate_uncertainty": source["observation"]["retains_pseudorange_rate_uncertainty_mps"] is False,
        "fgo_does_not_consume_rate_uncertainty": source["fgo"]["consumes_raw_rate_uncertainty"] is False,
        "fgo_fixed_doppler_sigma_path_present": source["fgo"]["uses_fixed_undifferenced_doppler_sigma"] and source["fgo"]["uses_fixed_single_difference_doppler_sigma"],
        "next_factor_raw_rate_uncertainty_not_cli_exposed": source["cli"]["raw_rate_uncertainty_option"] is False,
        "android_rate_sign_contract_present": source["doppler_contract"]["android_rate_to_doppler_sign"],
    }
    gates = {
        "phase46_artifact_hashes_exact": True,
        "phase46_clock_no_go_status_exact": policy["status"] == "no-go-clock-correction-common-mode-only",
        "phase46_bias_uncertainty_evidence_all_routes": route_evidence["all_bias_uncertainty_populated"],
        "phase46_same_epoch_clock_spread_zero": route_evidence["same_epoch_base_clock_spread_zero"],
        "phase46_same_epoch_signal_spread_zero": route_evidence["same_epoch_signal_spread_zero"],
        "phase46_geometry_changing_noncommon_effect_zero": route_evidence["noncommon_tow_effect_zero"],
        "bias_uncertainty_semantic_receiver_common": True,
        "bias_uncertainty_new_raw_audit_deduplicated": True,
        "source_next_factor_is_nonduplicate": all(source_checks.values()),
    }
    if not all(gates.values()):
        raise _fail("Phase56 dedup evidence gate failed: " + ", ".join(name for name, passed in gates.items() if not passed))
    reads: dict[str, Any] = {
        "single_process": True,
        "new_raw_device_gnss_reads": 0,
        "new_truth_reads": 0,
        "new_navigation_reads": 0,
        "new_solver_invocations": 0,
        "new_coordinate_inputs": 0,
        "new_imu_reads": 0,
        "phase46_sealed_artifact_reads": artifact_reads,
        "phase55_policy_record_read": 1,
        "phase46_raw_device_gnss_rereads": 0,
        "archive_reopens": 0,
        "rematerialization": 0,
        "validation_holdout": 0,
        "MAT": 0,
        "Kaggle_token": 0,
    }
    result = {
        "schema_version": SCHEMA,
        "phase": 56,
        "execution_label": "Luna Max",
        "status": "phase56-no-go-bias-uncertainty-duplicate-common-mode",
        "operation": "sealed Phase46 evidence-map/dedup audit; no new raw read",
        "factor_considered": "raw Android per-satellite BiasUncertaintyNanos/receiver-clock uncertainty relationship",
        "audit_only": True,
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "phase46_policy_view": policy,
        "phase46_evidence": route_evidence,
        "source_evidence": source,
        "source_checks": source_checks,
        "gates": {"all_passed": True, "observed": gates},
        "read_accounting": reads,
        "decision": {
            "bias_uncertainty_audit": "no-go-without-raw-deduplicated",
            "bias_uncertainty_is_new_geometry_factor": False,
            "native_correction_authorized": False,
            "new_raw_audit_authorized": False,
            "next_single_nonduplicate_factor": expected_next,
            "next_factor_mechanism": "opt-in native FGO Doppler sigma floor candidate using source-exact PseudorangeRateUncertaintyMetersPerSecond in m/s; no Phase56 implementation",
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "zero_point_782": "not evaluated without truth",
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    result_path = output_root / "phase56_bias_uncertainty_dedup.json"
    manifest_path = output_root / "phase56_bias_uncertainty_dedup.manifest.json"
    result_payload = _atomic_json(result_path, result)
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 56,
        "freeze_sha256": FREEZE_SHA256,
        "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "read_accounting": reads,
        "artifacts": {
            "result": {"path": _relative(result_path), "bytes": len(result_payload), "sha256": _sha256_bytes(result_payload)},
        },
    }
    _atomic_json(manifest_path, output_manifest)
    return result


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify freeze and evaluator manifest without reading sealed artifacts")
    parser.add_argument("--audit", action="store_true", help="publish evidence-map/dedup result from sealed non-raw artifacts")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.verify_freeze == args.audit:
        parser.error("choose exactly one of --verify-freeze or --audit")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase56 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({"status": result["status"], "new_raw_reads": result["read_accounting"]["new_raw_device_gnss_reads"], "next_factor": result["decision"]["next_single_nonduplicate_factor"]}, sort_keys=True))
        return 0
    except Phase56Error as exc:
        print(f"phase56 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
