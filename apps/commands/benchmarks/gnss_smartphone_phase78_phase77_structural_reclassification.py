#!/usr/bin/env python3
"""Reclassify the sealed Phase77 matrix without rerunning native code.

Phase77's candidate artifacts are immutable inputs here.  This scorer verifies
their hashes, summary telemetry, repeat identity, prediction-key identity,
finite-pc accounting, and speed/epoch/IMU invariants.  It never opens raw
GNSS/IMU/navigation/base/truth/MAT/archive data and never invokes a solver.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase78_phase77_structural_reclassification_freeze_v1.json"
FREEZE_SHA256 = "0214783abba6584383085c00824a3c227d7e17fdc041e369fcdd369e48baf897"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase78_phase77_structural_reclassification_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
PHASE77_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_result_v1.json"
PHASE77_RESULT_SHA256 = "e88ee525466432e1ba39b966ae0700b05ae41938ecee131097912ddff94c6ded"
PHASE77_FAILURE = ROOT / "output/smartphone-r5/phase77-phase73-signal-bias-composition-structural-v1/phase77_phase73_signal_bias_composition_structural_failure.json"
PHASE77_FAILURE_SHA256 = "54a11a86c1900c77e23ca07a32875912ba67033e635eda128e08a8ff4f9dc1ea"
PHASE77_FAILURE_BYTES = 437248
OUTPUT_ROOT = ROOT / "output/smartphone-r5/phase78-phase77-structural-reclassification-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase78-phase77-structural-reclassification-result.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)


class Phase78StructuralError(ValueError):
    """Raised when a sealed Phase77 artifact fails reclassification."""


def fail(message: str) -> Phase78StructuralError:
    return Phase78StructuralError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    if any(term in token for term in ("ground_truth", "validation", "holdout", "kaggle", "token")):
        raise fail(f"forbidden path: {path}")


def sha256(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing artifact: {path}")
    import hashlib

    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    reject_forbidden(path)
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label}: {path}: {exc}") from exc
    if not isinstance(value, dict):
        raise fail(f"{label} is not an object: {path}")
    return value


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    temporary.replace(path)


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise fail("Phase78 freeze hash changed")
    freeze = load_json(FREEZE, "Phase78 freeze")
    if freeze.get("status") != "frozen-before-phase78-sealed-artifact-read":
        raise fail("Phase78 freeze status changed")
    if freeze.get("scope", {}).get("no_native_rerun") is not True or freeze.get("scope", {}).get("no_accuracy_truth") is not True or freeze.get("scope", {}).get("no_route_specific_selection") is not True:
        raise fail("Phase78 sealed-only scope changed")
    if tuple(freeze.get("scope", {}).get("routes", ())) != ROUTES:
        raise fail("Phase78 route order changed")
    pin = freeze.get("authority", {}).get("phase77_result_record", {})
    if pin.get("sha256") != PHASE77_RESULT_SHA256 or sha256(PHASE77_RESULT) != PHASE77_RESULT_SHA256:
        raise fail("Phase77 result record pin changed")
    failure_pin = freeze.get("authority", {}).get("phase77_failure_output", {})
    if failure_pin.get("sha256") != PHASE77_FAILURE_SHA256 or failure_pin.get("bytes") != PHASE77_FAILURE_BYTES:
        raise fail("Phase77 failure output pin changed in freeze")
    gates = freeze.get("fixed_reclassification_gates", {})
    for key in (
        "candidate_artifact_hashes_exact",
        "candidate_repeat_submission_and_summary_identity",
        "candidate_prediction_domain_coverage",
        "retained_finite_pc_fraction_each_route",
        "pseudorange_factor_count_equals_retained",
        "tdcp_built_equals_inserted",
        "signal_bias_states_and_factors_material_each_route",
        "signal_bias_estimates_all_finite",
        "all_coordinates_finite_and_earth_valid",
        "output_epoch_and_imu_repeat_identity",
        "no_route_specific_selection",
        "truth_free",
        "all_gates_anded",
    ):
        expected = 1.0 if key == "candidate_prediction_domain_coverage" else True
        if gates.get(key) != expected:
            raise fail(f"Phase78 gate declaration changed: {key}")
    if gates.get("speed_over_70_count") != 0 or gates.get("native_reruns") != 0 or gates.get("raw_reads") != 0:
        raise fail("Phase78 zero-read gate declaration changed")
    accounting = freeze.get("read_accounting_at_freeze", {})
    if any(accounting.get(key) != 0 for key in ("phase78_sealed_artifact_reads", "raw_device_gnss", "raw_device_imu", "broadcast_nav", "base_rinex", "truth", "mat", "native_solver_invocations")):
        raise fail("Phase78 freeze read accounting changed")
    if not MANIFEST.is_file():
        raise fail("Phase78 manifest is missing")
    manifest = load_json(MANIFEST, "Phase78 manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("routes") != list(ROUTES):
        raise fail("Phase78 manifest freeze/routes pin changed")
    if manifest.get("read_accounting", {}).get("native_reruns") != 0 or manifest.get("read_accounting", {}).get("raw_reads") != 0 or manifest.get("read_accounting", {}).get("truth_reads") != 0:
        raise fail("Phase78 manifest read contract changed")
    if manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256(EVALUATOR):
        raise fail("Phase78 evaluator manifest pin changed")
    return freeze


def _finite(value: Any, label: str) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise fail(f"nonfinite value: {label}")
    if isinstance(value, dict):
        for key, item in value.items():
            _finite(item, f"{label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _finite(item, f"{label}[{index}]")


def _read_prediction(path: Path, route: str) -> list[tuple[int, float, float]]:
    reject_forbidden(path)
    lines = path.read_text(encoding="utf-8-sig").splitlines()
    if not lines or lines[0] != "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees":
        raise fail(f"submission header mismatch: {path}")
    rows: list[tuple[int, float, float]] = []
    previous: int | None = None
    for line_number, line in enumerate(lines[1:], start=2):
        fields = line.split(",")
        if len(fields) != 4 or fields[0] != route:
            raise fail(f"submission key mismatch: {path}:{line_number}")
        try:
            timestamp = int(fields[1])
            latitude = float(fields[2])
            longitude = float(fields[3])
        except ValueError as exc:
            raise fail(f"non-numeric submission row: {path}:{line_number}") from exc
        if previous is not None and timestamp <= previous:
            raise fail(f"submission timestamps are not increasing: {path}")
        if not all(math.isfinite(item) for item in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"invalid coordinate: {path}:{line_number}")
        rows.append((timestamp, latitude, longitude))
        previous = timestamp
    if not rows:
        raise fail(f"empty submission: {path}")
    return rows


def _speed_report(rows: list[tuple[int, float, float]]) -> dict[str, Any]:
    earth_radius_m = 6_371_000.0
    speeds: list[float] = []
    for previous, current in zip(rows, rows[1:]):
        dt = (current[0] - previous[0]) / 1000.0
        if dt <= 0.0:
            raise fail("non-positive submission interval")
        lat1, lon1, lat2, lon2 = map(math.radians, (previous[1], previous[2], current[1], current[2]))
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        hav = math.sin(dlat / 2.0) ** 2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2.0) ** 2
        distance = earth_radius_m * 2.0 * math.asin(math.sqrt(min(1.0, max(0.0, hav))))
        speeds.append(distance / dt)
    return {"finite": all(math.isfinite(speed) for speed in speeds), "max_mps": max(speeds, default=0.0), "over_70_mps_count": sum(speed > 70.0 for speed in speeds), "count": len(speeds)}


def _artifact(path: Path, expected_sha: str, route: str) -> dict[str, Any]:
    actual_sha = sha256(path)
    if actual_sha != expected_sha:
        raise fail(f"Phase77 candidate artifact hash changed: {route}/{path.name}")
    return {"path": relative(path), "bytes": path.stat().st_size, "sha256": actual_sha}


def _validate_summary(path: Path, route: str, expected: dict[str, Any]) -> dict[str, Any]:
    summary = load_json(path, "Phase77 candidate summary")
    _finite(summary, f"summary[{route}]")
    if summary.get("dataset_id") != route or summary.get("truth_used") is not False or summary.get("native_signal_bias_states") is not True:
        raise fail(f"Phase77 candidate summary contract failed: {route}")
    epochs = summary.get("epochs")
    graph = summary.get("graph")
    tdcp = summary.get("tdcp_contract")
    miss = summary.get("native_base_pseudorange_source_miss_mask")
    if not all(isinstance(value, dict) for value in (epochs, graph, tdcp, miss)):
        raise fail(f"Phase77 candidate telemetry missing: {route}")
    assert isinstance(epochs, dict) and isinstance(graph, dict) and isinstance(tdcp, dict) and isinstance(miss, dict)
    original = int(miss.get("original_adopted_pseudorange_rows", -1))
    retained = int(miss.get("retained_finite_pc_pseudorange_rows", -1))
    dropped_missing = int(miss.get("dropped_missing_exact_stream_rows", -1))
    dropped_domain = int(miss.get("dropped_out_of_domain_rows", -1))
    dropped_nonfinite = int(miss.get("dropped_nonfinite_correction_rows", -1))
    if original <= 0 or retained <= 0 or min(dropped_missing, dropped_domain, dropped_nonfinite) < 0 or original != retained + dropped_missing + dropped_domain + dropped_nonfinite:
        raise fail(f"Phase77 candidate miss-mask accounting failed: {route}")
    if miss.get("retained_finite_pc_fraction") != 1.0 or int(miss.get("pseudorange_factors_inserted", -1)) != retained or miss.get("pseudorange_factor_count_consistent") is not True:
        raise fail(f"Phase77 candidate finite-pc/factor gate failed: {route}")
    if retained != int(expected["retained_finite_pc_pseudorange_rows"]) or original != int(expected["original_adopted_pseudorange_rows"]) or dropped_missing != int(expected["dropped_missing_exact_stream_rows"]) or dropped_domain != int(expected["dropped_out_of_domain_rows"]):
        raise fail(f"Phase77 candidate telemetry pin mismatch: {route}")
    if int(epochs.get("pseudorange_factors", -1)) != retained or int(epochs.get("output", -1)) != int(expected["output_epochs"]) or int(graph.get("imu_intervals", -1)) != int(expected["imu_intervals"]):
        raise fail(f"Phase77 candidate epoch/IMU population mismatch: {route}")
    if int(tdcp.get("factors_built", -1)) != int(expected["tdcp_factors_built"]) or int(tdcp.get("factors_inserted", -1)) != int(expected["tdcp_factors_inserted"]) or int(tdcp.get("factors_built", -1)) != int(tdcp.get("factors_inserted", -2)) or int(tdcp.get("nonfinite_residuals", -1)) != 0:
        raise fail(f"Phase77 candidate TDCP accounting mismatch: {route}")
    states = int(epochs.get("receiver_signal_bias_states", -1))
    factors = int(epochs.get("receiver_signal_bias_factors", -1))
    estimates = summary.get("receiver_signal_bias_estimates_m")
    if states < 1 or factors < 1 or factors < states or not isinstance(estimates, dict) or len(estimates) != states:
        raise fail(f"Phase77 candidate signal-bias materiality failed: {route}")
    if states != int(expected["receiver_signal_bias_states"]) or factors != int(expected["receiver_signal_bias_factors"]) or estimates != expected["receiver_signal_bias_estimates_m"]:
        raise fail(f"Phase77 candidate signal-bias telemetry pin mismatch: {route}")
    for key, value in estimates.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            raise fail(f"nonfinite Phase77 signal-bias estimate: {route}/{key}")
    return {
        "miss_mask": {key: miss.get(key) for key in ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_finite_pc_fraction")},
        "population": {"pseudorange_factors": epochs.get("pseudorange_factors"), "tdcp_factors_built": tdcp.get("factors_built"), "tdcp_factors_inserted": tdcp.get("factors_inserted"), "imu_intervals": graph.get("imu_intervals"), "output_epochs": epochs.get("output")},
        "signal_bias": {"states": states, "factors": factors, "estimates_m": estimates},
    }


def _candidate_case(root: Path, route: str, run_number: int, pin: dict[str, Any], expected: dict[str, Any]) -> dict[str, Any]:
    run_dir = root / route / "candidate" / f"run{run_number}"
    submission = run_dir / "submission.csv"
    summary = run_dir / "summary.json"
    if not submission.is_file() or not summary.is_file():
        raise fail(f"Phase77 candidate artifact missing: {route}/run{run_number}")
    submission_artifact = _artifact(submission, pin["submission_sha256"], route)
    summary_artifact = _artifact(summary, pin["summary_sha256"], route)
    rows = _read_prediction(submission, route)
    speed = _speed_report(rows)
    if not speed["finite"] or speed["over_70_mps_count"] != 0:
        raise fail(f"Phase77 candidate speed gate failed: {route}/run{run_number}")
    telemetry = _validate_summary(summary, route, expected)
    return {"submission_artifact": submission_artifact, "summary_artifact": summary_artifact, "prediction_keys": [row[0] for row in rows], "rows": len(rows), "speed": speed, **telemetry}


def run_reclassification(output_root: Path = OUTPUT_ROOT) -> dict[str, Any]:
    freeze = verify_freeze()
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase78 output: {output_root}")
    failure = load_json(PHASE77_FAILURE, "Phase77 failure output")
    if sha256(PHASE77_FAILURE) != PHASE77_FAILURE_SHA256 or PHASE77_FAILURE.stat().st_size != PHASE77_FAILURE_BYTES:
        raise fail("Phase77 failure output hash/bytes changed")
    pins = freeze.get("candidate_artifact_pins", {})
    telemetry_pins = freeze.get("candidate_telemetry_pins", {})
    if set(pins) != set(ROUTES) or set(telemetry_pins) != set(ROUTES):
        raise fail("Phase78 candidate route pins are incomplete")
    routes: dict[str, Any] = {}
    try:
        for route in ROUTES:
            if route not in failure.get("routes", {}):
                raise fail(f"Phase77 failure route missing: {route}")
            expected = telemetry_pins[route]
            run1 = _candidate_case(ROOT / "output/smartphone-r5/phase77-phase73-signal-bias-composition-structural-v1", route, 1, pins[route]["candidate_run1"], expected)
            run2 = _candidate_case(ROOT / "output/smartphone-r5/phase77-phase73-signal-bias-composition-structural-v1", route, 2, pins[route]["candidate_run2"], expected)
            repeat_identity = (
                run1["submission_artifact"]["sha256"] == run2["submission_artifact"]["sha256"]
                and run1["summary_artifact"]["sha256"] == run2["summary_artifact"]["sha256"]
                and run1["prediction_keys"] == run2["prediction_keys"]
                and run1["miss_mask"] == run2["miss_mask"]
                and run1["population"] == run2["population"]
                and run1["signal_bias"] == run2["signal_bias"]
            )
            if not repeat_identity:
                raise fail(f"Phase77 candidate repeat identity failed: {route}")
            routes[route] = {"candidate_run1": {key: value for key, value in run1.items() if key != "prediction_keys"}, "candidate_run2": {key: value for key, value in run2.items() if key != "prediction_keys"}, "prediction_domain": {"rows": run1["rows"], "keys_repeat_identity": True}, "repeat_identity": True, "gates": {"candidate_artifact_hashes_exact": True, "retained_finite_pc_fraction": True, "pseudorange_factor_count_equals_retained": True, "tdcp_built_equals_inserted": True, "signal_bias_material_and_finite": True, "speed_over_70_count_zero": True, "epoch_imu_repeat_identity": run1["population"]["imu_intervals"] == run2["population"]["imu_intervals"] and run1["population"]["output_epochs"] == run2["population"]["output_epochs"]}}
        result = {"schema_version": OUTPUT_SCHEMA, "phase": 78, "execution_label": "Luna Max", "status": "go-phase78-sealed-artifact-reclassification", "truth_free": True, "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "manifest": {"path": relative(MANIFEST), "sha256": sha256(MANIFEST)}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256(EVALUATOR)}, "phase77_failure": {"path": relative(PHASE77_FAILURE), "sha256": PHASE77_FAILURE_SHA256, "bytes": PHASE77_FAILURE_BYTES}, "routes": routes, "gates": {"all_four_routes": True, "candidate_artifact_hashes_exact": True, "candidate_repeat_identity": True, "candidate_prediction_domain_coverage": 1.0, "retained_finite_pc_fraction_each_route": 1.0, "pseudorange_factor_count_equals_retained": True, "tdcp_built_equals_inserted": True, "signal_bias_states_and_factors_material_each_route": True, "signal_bias_estimates_all_finite": True, "all_coordinates_finite_and_earth_valid": True, "speed_over_70_count_zero": True, "output_epoch_and_imu_repeat_identity": True, "no_route_specific_selection": True, "truth_free": True, "all_passed": True}, "read_accounting": {"phase78_sealed_artifact_reads": "Phase77 candidate submission/summary and failure artifacts only", "native_reruns": 0, "raw_reads": 0, "truth_reads": 0, "mat_reads_or_generated": 0, "validation_holdout_reads": 0, "kaggle_token_reads": 0, "archive_reopens": 0}, "phase43_champion_preserved": True, "phase77_experimental_preserved": True, "accuracy_authorized_next": True, "zero_point_782_claim": "not evaluated"}
        result_path = output_root / "phase78_phase77_structural_reclassification_result.json"
        atomic_json(result_path, result)
        atomic_json(output_root / "phase78_phase77_structural_reclassification_manifest.json", {"schema_version": "smartphone-r5-phase78-phase77-structural-reclassification-output-manifest.v1", "phase": 78, "status": "sealed-artifact-only", "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "result": {"path": relative(result_path), "sha256": sha256(result_path), "bytes": result_path.stat().st_size}, "native_reruns": 0, "raw_reads": 0, "truth_reads": 0, "all_gates_passed": True})
        return result
    except Exception as exc:
        failure_record = {"schema_version": "smartphone-r5-phase78-phase77-structural-reclassification-failure.v1", "status": "fail-closed", "error": str(exc), "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "routes": routes, "native_reruns": 0, "raw_reads": 0, "truth_reads": 0}
        atomic_json(output_root / "phase78_phase77_structural_reclassification_failure.json", failure_record)
        raise


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run", action="store_true")
    parser.add_argument("--output-root", type=Path, default=OUTPUT_ROOT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run:
            result = run_reclassification(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["gates"]["all_passed"], "native_reruns": result["read_accounting"]["native_reruns"], "raw_reads": result["read_accounting"]["raw_reads"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run is required")
        return 0
    except Phase78StructuralError as exc:
        print(f"phase78 reclassification failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
