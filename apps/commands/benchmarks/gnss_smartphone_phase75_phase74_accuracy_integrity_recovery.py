#!/usr/bin/env python3
"""Scorer-only recovery of the Phase74 truth-header evaluator defect.

Phase75 changes no candidate, control, metric, or accuracy gate.  It imports
only pure parsing/metric helpers from the sealed Phase74 scorer, reads the
immutable Phase73 structural run1 artifacts once each, and reads each declared
Phase44 development truth once.  No native process, solver, raw input,
archive, MATLAB product, validation, holdout, WLS, or Kaggle data is opened.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PHASE74_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"
_SPEC = importlib.util.spec_from_file_location("phase74_accuracy_helpers", PHASE74_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase74 helper: {PHASE74_PATH}")
P74 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P74)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "47dcbaa1f2e7ec8d43e87905d29a2d7fd9c520327516db8c3c871d62fd67b663"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase75-phase74-accuracy-integrity-recovery-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase75-phase74-accuracy-integrity-recovery-result.v1"
PHASE74_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_freeze_v1.json"
PHASE74_FREEZE_SHA256 = "8308f15a491d1ef1daae0542efd1d7ac96562738e51e6c90ea04fd9f81aba8d9"
PHASE74_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_failure_v1.json"
PHASE74_FAILURE_SHA256 = "aa03f91742e480727d7f52a45c697afeb20750b71f6cb45212da6aeec9e6cec9"
PHASE74_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_manifest_v1.json"
PHASE74_MANIFEST_SHA256 = "6a80232cf380ab85eee531d07654563173d8dfb7808f457870c9464e7ed91373"
PHASE74_EVALUATOR_SHA256 = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"
PHASE73_RESULT_RECORD = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_result_v1.json"
PHASE73_RESULT_RECORD_SHA256 = "8493d16b3d2c5ce8b09b65ab0dada6903fcbac01257fb8db7977cb41c85577fd"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
EARTH_RADIUS_M = 6_371_008.8
MAX_SPEED_MPS = 70.0
PHASE43_CONTROL = {
    ROUTES[0]: {
        "submission": "d4d7652e5d12389466e586fe2d8e85d34977a7e11036cee2f591a89293c5426c",
        "summary": "d4980260e257c92d7e2fb716f900501fd55c3eb9278f9b38a4f72ac2e62303fe",
    },
    ROUTES[1]: {
        "submission": "3cfe97750927fa268f71bbe7393754ce7a1eb39d937e085b177c50a71eec10ae",
        "summary": "23f409e5708dc547bc86b156e07a6fb305732cb9ce8eb3da57f26fd487e4404e",
    },
    ROUTES[2]: {
        "submission": "4eb2b566708a87db4903610a41cd648c7ff065d35710d358894a78eaa8e116e3",
        "summary": "f4fb3bc2b09d44700ee83772d4b23ad93b68d07eff6743b9525626f805264eb8",
    },
    ROUTES[3]: {
        "submission": "524769cdf67aa857eefaafdadf943dd76586dda6a53e8b6d1df4a707d7699f71",
        "summary": "eeb4f60e352ea00277604516b432673317ab035daa601135c6e9f989cb118926",
    },
}


class Phase75RecoveryError(ValueError):
    """Raised when the Phase75 recovery contract fails."""


def fail(message: str) -> Phase75RecoveryError:
    return Phase75RecoveryError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("validation", "holdout", "kaggle", "token", "device_wls", "svposition", "svelevation"):
        if term in token:
            raise fail(f"forbidden Phase75 path: {path}")


def sha256_file(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
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


def atomic_write(path: Path, payload: bytes) -> None:
    reject_forbidden(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with open(descriptor, "wb", closefd=True) as handle:
            handle.write(payload)
            handle.flush()
        Path(temporary).replace(path)
        temporary = ""
    finally:
        if temporary:
            try:
                Path(temporary).unlink()
            except FileNotFoundError:
                pass


def atomic_json(path: Path, value: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _read_once(path: Path, expected_sha256: str, expected_bytes: int | None, label: str) -> bytes:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    with path.open("rb") as handle:
        payload = handle.read()
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise fail(f"{label} byte-size mismatch: {path}")
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase75 freeze hash changed")
    freeze = load_json(FREEZE, "Phase75 recovery freeze")
    if freeze.get("status") != "frozen-before-phase75-truth-read":
        raise fail("Phase75 freeze status changed")
    authority = freeze.get("authority", {})
    pins = {
        "phase74_failure_record": (PHASE74_FAILURE, PHASE74_FAILURE_SHA256),
        "phase74_freeze": (PHASE74_FREEZE, PHASE74_FREEZE_SHA256),
        "phase74_manifest": (PHASE74_MANIFEST, PHASE74_MANIFEST_SHA256),
        "phase73_structural_result": (PHASE73_RESULT_RECORD, PHASE73_RESULT_RECORD_SHA256),
    }
    for key, (path, expected) in pins.items():
        if authority.get(key, {}).get("sha256") != expected or sha256_file(path) != expected:
            raise fail(f"sealed authority pin changed: {key}")
    if authority.get("phase74_evaluator", {}).get("sha256") != PHASE74_EVALUATOR_SHA256 or sha256_file(PHASE74_PATH) != PHASE74_EVALUATOR_SHA256:
        raise fail("Phase74 evaluator pin changed")
    scope = freeze.get("recovery_scope", {})
    if scope.get("allowed_change") != "truth parser field-name handling only" or scope.get("forbidden_change") != "metric, gates, artifact paths/hashes, candidate/control data, native algorithm, solver, or input cohort" or scope.get("prior_phase74_output_reuse") is not False:
        raise fail("Phase75 recovery scope changed")
    parser = freeze.get("truth_parser_contract", {})
    if parser.get("reader") != "CSV DictReader over header names" or parser.get("required_fields") != ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"] or parser.get("phone_policy") != "if phone column exists it must equal the declared route; if absent assign the declared route":
        raise fail("Phase75 truth parser contract changed")
    metric = freeze.get("metric_contract", {})
    if metric.get("earth_radius_m") != EARTH_RADIUS_M or metric.get("percentile") != "linear interpolation at rank (n - 1) * q" or metric.get("score") != "(P50 + P95) / 2" or metric.get("prediction_domain_coverage") != 1.0:
        raise fail("Phase75 metric contract changed")
    gates = freeze.get("accuracy_gates", {})
    required_gates = {
        "unchanged_from_phase74": True,
        "candidate_improves_each_route_by_at_least_m": 0.05,
        "candidate_no_route_regression": True,
        "candidate_macro_improvement_at_least_m": 0.1,
        "candidate_mtv_h_improvement_at_least_m": 0.1,
        "candidate_macro_score_max_m": 2.0,
        "candidate_route_score_max_m": 3.0,
        "candidate_mtv_h_p95_max_m": 5.0,
        "candidate_all_finite": True,
        "candidate_over_70_mps_count": 0,
    }
    if any(gates.get(key) != value for key, value in required_gates.items()):
        raise fail("Phase75 accuracy gates changed")
    read_policy = freeze.get("read_policy", {})
    for key, value in (("truth_reads_before_freeze", 0), ("truth_reads_before_manifest", 0), ("truth_reads_after_manifest", 4), ("truth_reads_per_route", 1), ("candidate_artifact_reads", 8), ("control_artifact_reads", 8)):
        if read_policy.get(key) != value:
            raise fail(f"Phase75 read contract changed: {key}")
    if read_policy.get("native_or_solver_subprocess") is not False or read_policy.get("post_truth_tuning") is not False:
        raise fail("Phase75 native/post-truth policy changed")
    if tuple(freeze.get("cohort", {}).get("route_order", ())) != ROUTES:
        raise fail("Phase75 route order changed")
    if set(freeze.get("cohort", {}).get("truths", {})) != set(ROUTES):
        raise fail("Phase75 truth route set changed")
    return freeze


def _parse_truth_dictreader(payload: bytes, route: str) -> dict[int, tuple[float, float]]:
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise fail(f"truth is not UTF-8: {route}") from exc
    reader = csv.DictReader(io.StringIO(text, newline=""))
    fields = list(reader.fieldnames or ())
    required = ("UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees")
    if any(field not in fields for field in required):
        raise fail(f"truth missing required fields: {route}")
    if len(fields) != len(set(fields)):
        raise fail(f"truth header has duplicate fields: {route}")
    has_phone = "phone" in fields
    result: dict[int, tuple[float, float]] = {}
    for row_number, raw in enumerate(reader, start=2):
        if None in raw:
            raise fail(f"truth has unnamed extra cells: {route}:{row_number}")
        phone = (raw.get("phone") or route) if has_phone else route
        if phone != route or phone != phone.strip():
            raise fail(f"truth phone mismatch: {route}:{row_number}")
        try:
            timestamp = int((raw.get("UnixTimeMillis") or "").strip())
            latitude = float((raw.get("LatitudeDegrees") or "").strip())
            longitude = float((raw.get("LongitudeDegrees") or "").strip())
        except (TypeError, ValueError) as exc:
            raise fail(f"truth numeric field invalid: {route}:{row_number}") from exc
        if timestamp < 0 or not all(math.isfinite(value) for value in (latitude, longitude)) or not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
            raise fail(f"truth coordinate invalid: {route}:{row_number}")
        if timestamp in result:
            raise fail(f"duplicate truth key: {route}:{timestamp}")
        result[timestamp] = (latitude, longitude)
    if not result:
        raise fail(f"truth has no rows: {route}")
    return result


def _score_prediction(prediction: dict[int, tuple[float, float]], truth: dict[int, tuple[float, float]], expected_missing: list[Any] | None, route: str, ordered: list[tuple[int, float, float]]) -> dict[str, Any]:
    return P74._score_prediction(prediction, truth, expected_missing, route, ordered)


def _artifact_pin(freeze74: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    try:
        route_pin = freeze74["sealed_phase73_artifacts"]["routes"][route]
        return route_pin["candidate"], route_pin["control"]
    except (KeyError, TypeError) as exc:
        raise fail(f"Phase73 artifact pin missing: {route}") from exc


def _phase43_control_identity(route: str, control_pin: dict[str, Any]) -> bool:
    expected = PHASE43_CONTROL[route]
    return control_pin["submission_sha256"] == expected["submission"] and control_pin["summary_sha256"] == expected["summary"]


def score_route(freeze: dict[str, Any], freeze74: dict[str, Any], route: str, accounting: dict[str, int]) -> dict[str, Any]:
    candidate_pin, control_pin = _artifact_pin(freeze74, route)
    candidate_submission_path = ROOT / candidate_pin["submission_path"]
    candidate_summary_path = ROOT / candidate_pin["summary_path"]
    control_submission_path = ROOT / control_pin["submission_path"]
    control_summary_path = ROOT / control_pin["summary_path"]
    accounting["candidate_artifact_reads"] += 1
    candidate_submission = _read_once(candidate_submission_path, candidate_pin["submission_sha256"], int(candidate_pin["submission_bytes"]), "candidate submission")
    accounting["candidate_artifact_reads"] += 1
    candidate_summary = _read_once(candidate_summary_path, candidate_pin["summary_sha256"], None, "candidate summary")
    accounting["control_artifact_reads"] += 1
    control_submission = _read_once(control_submission_path, control_pin["submission_sha256"], int(control_pin["submission_bytes"]), "control submission")
    accounting["control_artifact_reads"] += 1
    control_summary = _read_once(control_summary_path, control_pin["summary_sha256"], None, "control summary")
    candidate_rows = P74._parse_submission(candidate_submission, route)[0]
    candidate_map = P74._parse_submission(candidate_submission, route)[1]
    control_rows = P74._parse_submission(control_submission, route)[0]
    control_map = P74._parse_submission(control_submission, route)[1]
    candidate_diag = P74._validate_summary(candidate_summary, route, True)
    P74._validate_summary(control_summary, route, False)
    if candidate_map.keys() != control_map.keys():
        raise fail(f"candidate/control prediction keys differ: {route}")
    truth_pin = freeze["cohort"]["truths"][route]
    truth_path = ROOT / truth_pin["path"]
    accounting["truth_reads"] += 1
    truth_payload = _read_once(truth_path, truth_pin["sha256"], int(truth_pin["bytes"]), "development truth")
    truth_map = _parse_truth_dictreader(truth_payload, route)
    if len(truth_map) != int(truth_pin["rows"]):
        raise fail(f"truth row count mismatch: {route}")
    candidate_score = _score_prediction(candidate_map, truth_map, truth_pin.get("expected_missing_key"), route, candidate_rows)
    control_score = _score_prediction(control_map, truth_map, truth_pin.get("expected_missing_key"), route, control_rows)
    improvement = control_score["score_m"] - candidate_score["score_m"]
    return {
        "truth": {"path": relative(truth_path), "sha256": truth_pin["sha256"], "bytes": len(truth_payload), "rows": len(truth_map), "read_count": 1, "expected_missing_truth_rows": truth_pin["expected_missing_truth_rows"], "expected_missing_key": truth_pin.get("expected_missing_key")},
        "artifact_hashes": {"candidate_submission": candidate_pin["submission_sha256"], "candidate_summary": candidate_pin["summary_sha256"], "control_submission": control_pin["submission_sha256"], "control_summary": control_pin["summary_sha256"]},
        "candidate": candidate_score,
        "control": control_score,
        "improvement_m": improvement,
        "candidate_phase73_telemetry": {key: candidate_diag["miss_mask"][key] for key in ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_over_original_fraction", "correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m")},
        "control_phase43_identity": _phase43_control_identity(route, control_pin),
    }


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    freeze74 = load_json(PHASE74_FREEZE, "Phase74 accuracy freeze")
    if sha256_file(PHASE74_FREEZE) != PHASE74_FREEZE_SHA256:
        raise fail("Phase74 accuracy freeze bytes changed")
    if not MANIFEST.is_file():
        raise fail("Phase75 recovery manifest is missing")
    manifest = load_json(MANIFEST, "Phase75 recovery manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256_file(EVALUATOR) or manifest.get("read_policy", {}).get("truth_reads") != 4 or manifest.get("read_policy", {}).get("native_or_solver_subprocess") is not False:
        raise fail("Phase75 manifest pin/read contract changed")
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase75 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    accounting = {"candidate_artifact_reads": 0, "control_artifact_reads": 0, "truth_reads": 0}
    routes: dict[str, Any] = {}
    try:
        for route in ROUTES:
            report = score_route(freeze, freeze74, route, accounting)
            candidate = report["candidate"]
            improvement = report["improvement_m"]
            route_gate = {
                "candidate_improvement_at_least_0_05m": improvement >= 0.05,
                "candidate_no_route_regression": improvement >= 0.0,
                "candidate_prediction_domain_coverage_exact": candidate["prediction_domain_coverage"] == 1.0,
                "candidate_finite": candidate["finite"],
                "candidate_over_70_mps_count_zero": candidate["over_70_mps_count"] == 0,
                "candidate_route_score_at_most_3m": candidate["score_m"] <= 3.0,
                "control_phase43_identity": report["control_phase43_identity"],
            }
            report["gate"] = {"passed": all(route_gate.values()), "failures": [key for key, value in route_gate.items() if not value]}
            routes[route] = report
        candidate_scores = [routes[route]["candidate"]["score_m"] for route in ROUTES]
        control_scores = [routes[route]["control"]["score_m"] for route in ROUTES]
        candidate_macro = sum(candidate_scores) / len(candidate_scores)
        control_macro = sum(control_scores) / len(control_scores)
        macro_improvement = control_macro - candidate_macro
        target_route = ROUTES[1]
        gates = {
            "all_four_routes": len(routes) == 4,
            "candidate_each_route_improves_0_05m": all(routes[route]["gate"]["passed"] for route in ROUTES),
            "candidate_macro_improvement_at_least_0_10m": macro_improvement >= 0.1,
            "candidate_mtv_h_improvement_at_least_0_10m": routes[target_route]["improvement_m"] >= 0.1,
            "candidate_macro_score_at_most_2m": candidate_macro <= 2.0,
            "candidate_each_route_score_at_most_3m": all(routes[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES),
            "candidate_mtv_h_p95_at_most_5m": routes[target_route]["candidate"]["p95_m"] <= 5.0,
            "candidate_prediction_domain_coverage_exact": all(routes[route]["candidate"]["prediction_domain_coverage"] == 1.0 for route in ROUTES),
            "candidate_over_70_mps_count_zero": all(routes[route]["candidate"]["over_70_mps_count"] == 0 for route in ROUTES),
            "all_finite": all(routes[route]["candidate"]["finite"] and routes[route]["control"]["finite"] for route in ROUTES),
            "control_phase43_identity": all(routes[route]["control_phase43_identity"] for route in ROUTES),
        }
        all_passed = all(gates.values())
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 75,
            "execution_label": "Luna Max",
            "status": "go-phase75-accuracy-gates" if all_passed else "no-go-phase75-accuracy-gates",
            "decision": "candidate may proceed to separately frozen validation" if all_passed else "preserve-phase43-champion; keep-phase73-experimental; no-validation",
            "recovery_only": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR), "single_process": True, "native_or_solver_subprocess": False, "post_truth_tuning": False},
            "phase74_integrity_failure": {"path": relative(PHASE74_FAILURE), "sha256": PHASE74_FAILURE_SHA256, "truth_reads_previous_attempt": 1, "accuracy_observed_previous_attempt": False},
            "phase73_structural_input": {"result_record": {"path": relative(PHASE73_RESULT_RECORD), "sha256": PHASE73_RESULT_RECORD_SHA256}, "candidate_control_source": "Phase73 structural run1 artifacts only; no native rerun"},
            "metric_contract": freeze["metric_contract"],
            "routes": routes,
            "aggregate": {"candidate_macro_score_m": candidate_macro, "control_phase43_macro_score_m": control_macro, "macro_improvement_m": macro_improvement, "candidate_route_count": len(candidate_scores), "control_route_count": len(control_scores)},
            "historical_phase51_reference": freeze74["authority"]["phase52_historical_recovery_reference"],
            "phase44_baseline_reference": freeze74["authority"]["phase44_baseline_reference"],
            "accuracy_gates": gates | {"all_passed": all_passed},
            "stretch_0_782": {"target_score_m": 0.782, "candidate_macro_score_m": candidate_macro, "met": candidate_macro <= 0.782, "report_only": True},
            "read_accounting": {"single_scorer_process": True, "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "raw_gnss_reads": 0, "raw_imu_reads": 0, "navigation_reads": 0, "native_or_solver_subprocess": 0, "validation_holdout_reads": 0, "mat_reads_or_generated": 0, "device_wls_or_precomputed_coordinates": 0, "kaggle_or_token_access": 0, "archive_reopen_or_rematerialize": False, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase58_experimental_preserved": True,
            "fresh_validation": "not run in Phase75; separate freeze required" if all_passed else "not authorized",
            "zero_point_782_claim": "reported only; no promotion or Kaggle action",
        }
        result_path = output_root / "phase75_phase74_accuracy_integrity_recovery_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": "smartphone-r5-phase75-phase74-accuracy-integrity-recovery-output-manifest.v1", "phase": 75, "status": result["status"], "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256_file(result_path), "bytes": result_path.stat().st_size}, "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "native_or_solver_subprocess": 0, "all_gates_passed": all_passed}
        atomic_json(output_root / "phase75_phase74_accuracy_integrity_recovery_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase75-phase74-accuracy-integrity-recovery-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "routes_completed": sorted(routes), "native_or_solver_subprocess": 0, "post_truth_tuning": False}
        atomic_json(output_root / "phase75_phase74_accuracy_integrity_recovery_failure.json", failure)
        if isinstance(exc, Phase75RecoveryError):
            raise
        raise fail(f"unexpected scorer exception: {type(exc).__name__}: {exc}") from exc


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--run-score", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        if args.verify_freeze:
            verify_freeze()
        if args.run_score:
            result = run_score(args.output_root)
            print(json.dumps({"status": result["status"], "all_gates_passed": result["accuracy_gates"]["all_passed"], "truth_reads": result["read_accounting"]["truth_reads"]}, sort_keys=True))
        elif not args.verify_freeze:
            parser.error("one of --verify-freeze or --run-score is required")
        return 0
    except Phase75RecoveryError as exc:
        print(f"phase75 recovery failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
