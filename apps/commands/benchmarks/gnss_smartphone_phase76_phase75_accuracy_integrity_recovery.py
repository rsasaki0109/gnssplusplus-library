#!/usr/bin/env python3
"""Phase76 scorer-only recovery of the Phase75 control-identity defect.

Phase75 read one development truth and then failed closed because it looked
for a ``PHASE43_CONTROL`` helper constant that the sealed Phase74 module does
not export.  This recovery uses the immutable Phase74 freeze as the sole
source of control artifact identity.  It reads each Phase73 candidate/control
artifact and each declared Phase44 development truth once in one process; it
does not run native code or open raw, navigation, MATLAB, validation, or
holdout inputs.
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
_SPEC = importlib.util.spec_from_file_location("phase74_accuracy_helpers_phase76", PHASE74_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase74 helper: {PHASE74_PATH}")
P74 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P74)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_freeze_v1.json"
FREEZE_SHA256 = "6674773e4d988644f205b52bdae5228dcaa0160ac84a58f63d5192035ee121c8"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase76-phase75-accuracy-integrity-recovery-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase76-phase75-accuracy-integrity-recovery-result.v1"
PHASE74_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_freeze_v1.json"
PHASE74_FREEZE_SHA256 = "8308f15a491d1ef1daae0542efd1d7ac96562738e51e6c90ea04fd9f81aba8d9"
PHASE74_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_manifest_v1.json"
PHASE74_MANIFEST_SHA256 = "6a80232cf380ab85eee531d07654563173d8dfb7808f457870c9464e7ed91373"
PHASE74_EVALUATOR_SHA256 = "bf07ff7ccb9f9efce5a759f2fabe57b7dc0c24ac5acdaf57c6ad9fc88a3fd761"
PHASE75_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_freeze_v1.json"
PHASE75_FREEZE_SHA256 = "47dcbaa1f2e7ec8d43e87905d29a2d7fd9c520327516db8c3c871d62fd67b663"
PHASE75_FAILURE = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_failure_v1.json"
PHASE75_FAILURE_SHA256 = "4d8fcec61976a6e15dc2fcbf7b5fe21f3bbb7cb28534b0a86132f680570da352"
PHASE75_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase75_phase74_accuracy_integrity_recovery.py"
PHASE75_PATCHED_EVALUATOR_SHA256 = "95e9bf69055759ec6ff70630daf66295ce0a3d4ecdf59402058d785376ef4fd9"
PHASE75_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase75_phase74_accuracy_integrity_recovery_manifest_v1.json"
PHASE75_MANIFEST_SHA256 = "94cf97325dd1704f96ca609c208853fda1b3d559f3e48229258f318c613027f3"
PHASE75_INITIAL_EVALUATOR_SHA256 = "6adf405b7f683aae258ac254aa778158f90c4a59d8536300123a2d9f18b6bf32"
PHASE75_INITIAL_MANIFEST_SHA256 = "4ed6f12c75a1c717c16ad22907b25dd3eef92cc1b50ab7c815db86dfaa8a15c7"
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


class Phase76RecoveryError(ValueError):
    """Raised when a frozen Phase76 scoring contract fails."""


def fail(message: str) -> Phase76RecoveryError:
    return Phase76RecoveryError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("validation", "holdout", "kaggle", "token", "device_wls", "svposition", "svelevation"):
        if term in token:
            raise fail(f"forbidden Phase76 path: {path}")


def sha256_file(path: Path) -> str:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise fail(f"failed to hash {path}: {exc}") from exc
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


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _read_once(path: Path, expected_sha256: str, expected_bytes: int | None, label: str) -> bytes:
    reject_forbidden(path)
    if not path.is_file():
        raise fail(f"missing {label}: {path}")
    try:
        with path.open("rb") as handle:
            payload = handle.read()
    except OSError as exc:
        raise fail(f"failed to read {label}: {path}: {exc}") from exc
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise fail(f"{label} byte-size mismatch: {path}")
    if hashlib.sha256(payload).hexdigest() != expected_sha256:
        raise fail(f"{label} SHA-256 mismatch: {path}")
    return payload


def _require_routes(value: dict[str, Any], label: str) -> None:
    routes = tuple(value.get("cohort", {}).get("route_order", ()))
    if routes != ROUTES:
        raise fail(f"{label} route order changed")
    if set(value.get("cohort", {}).get("truths", {})) != set(ROUTES):
        raise fail(f"{label} truth route set changed")


def _phase74_control_pin(freeze74: dict[str, Any], route: str) -> dict[str, Any]:
    """Return the control pin directly from the sealed Phase74 route map."""
    try:
        value = freeze74["sealed_phase73_artifacts"]["routes"][route]["control"]
    except (KeyError, TypeError) as exc:
        raise fail(f"Phase74 sealed control pin missing: {route}") from exc
    if not isinstance(value, dict):
        raise fail(f"Phase74 sealed control pin is not an object: {route}")
    for key in ("submission_path", "submission_sha256", "submission_bytes", "summary_path", "summary_sha256"):
        if key not in value:
            raise fail(f"Phase74 sealed control pin missing {key}: {route}")
    return value


def _phase74_artifact_pins(freeze74: dict[str, Any], route: str) -> tuple[dict[str, Any], dict[str, Any]]:
    try:
        route_pin = freeze74["sealed_phase73_artifacts"]["routes"][route]
        candidate = route_pin["candidate"]
        control = _phase74_control_pin(freeze74, route)
    except (KeyError, TypeError) as exc:
        raise fail(f"Phase74 sealed artifact pin missing: {route}") from exc
    if not isinstance(candidate, dict):
        raise fail(f"Phase74 sealed candidate pin is not an object: {route}")
    return candidate, control


def _control_identity_from_phase74(freeze74: dict[str, Any], route: str, control_pin: dict[str, Any]) -> bool:
    """Compare a control pin with the immutable Phase74 nested route pin.

    This deliberately has no Phase43 constant: Phase74 already sealed and
    verified the exact Phase43 control artifact identity.
    """
    expected = _phase74_control_pin(freeze74, route)
    keys = ("submission_path", "submission_sha256", "submission_bytes", "summary_path", "summary_sha256")
    return all(control_pin.get(key) == expected.get(key) for key in keys)


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase76 freeze hash changed")
    freeze = load_json(FREEZE, "Phase76 freeze")
    if freeze.get("status") != "frozen-before-phase76-truth-read":
        raise fail("Phase76 freeze status changed")
    authority = freeze.get("authority", {})
    phase75_freeze_pin = authority.get("phase75_freeze", {})
    if phase75_freeze_pin.get("sha256") != PHASE75_FREEZE_SHA256 or sha256_file(PHASE75_FREEZE) != PHASE75_FREEZE_SHA256:
        raise fail("Phase75 freeze pin changed")
    failure_pin = authority.get("phase75_initial_failure", {})
    if failure_pin.get("sha256") != PHASE75_FAILURE_SHA256 or sha256_file(PHASE75_FAILURE) != PHASE75_FAILURE_SHA256:
        raise fail("Phase75 failure record pin changed")
    initial = authority.get("phase75_initial_evaluator_manifest", {})
    if initial.get("evaluator_sha256_before_truth") != PHASE75_INITIAL_EVALUATOR_SHA256 or initial.get("manifest_sha256_before_truth") != PHASE75_INITIAL_MANIFEST_SHA256:
        raise fail("Phase75 initial evaluator provenance changed")
    patched = authority.get("phase75_post_read_patch", {})
    if patched.get("sha256") != PHASE75_PATCHED_EVALUATOR_SHA256 or sha256_file(PHASE75_PATH) != PHASE75_PATCHED_EVALUATOR_SHA256:
        raise fail("Phase75 post-read patch provenance changed")
    phase75_manifest = authority.get("phase75_initial_evaluator_manifest", {}).get("manifest_path")
    if phase75_manifest != relative(PHASE75_MANIFEST) or sha256_file(PHASE75_MANIFEST) != PHASE75_MANIFEST_SHA256:
        raise fail("Phase75 patched manifest provenance changed")
    phase74_pin = authority.get("phase74_freeze", {})
    if phase74_pin.get("sha256") != PHASE74_FREEZE_SHA256 or sha256_file(PHASE74_FREEZE) != PHASE74_FREEZE_SHA256:
        raise fail("Phase74 freeze pin changed")
    phase74_manifest_pin = authority.get("phase74_manifest", {})
    if phase74_manifest_pin.get("sha256") != PHASE74_MANIFEST_SHA256 or sha256_file(PHASE74_MANIFEST) != PHASE74_MANIFEST_SHA256:
        raise fail("Phase74 manifest pin changed")
    phase74_evaluator_pin = authority.get("phase74_evaluator", {})
    if phase74_evaluator_pin.get("sha256") != PHASE74_EVALUATOR_SHA256 or sha256_file(PHASE74_PATH) != PHASE74_EVALUATOR_SHA256:
        raise fail("Phase74 evaluator pin changed")
    structural_pin = authority.get("phase73_structural_result", {})
    if structural_pin.get("sha256") != PHASE73_RESULT_RECORD_SHA256 or sha256_file(PHASE73_RESULT_RECORD) != PHASE73_RESULT_RECORD_SHA256:
        raise fail("Phase73 structural result pin changed")
    _require_routes(freeze, "Phase76")
    parser = freeze.get("truth_parser_contract", {})
    if parser.get("reader") != "CSV DictReader over header names" or parser.get("required_fields") != ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"] or parser.get("phone_policy") != "if phone column exists it must equal the declared route; if absent assign the declared route":
        raise fail("Phase76 truth parser contract changed")
    metric = freeze.get("metric_contract", {})
    if metric.get("key") != "(phone, UnixTimeMillis)" or metric.get("earth_radius_m") != EARTH_RADIUS_M or metric.get("percentile") != "linear interpolation at rank (n - 1) * q" or metric.get("score") != "(P50 + P95) / 2" or metric.get("prediction_domain_coverage") != "matched prediction keys / prediction keys; required exactly 1.0":
        raise fail("Phase76 metric contract changed")
    gates = freeze.get("accuracy_gates", {})
    expected_gates = {
        "declared_before_truth": True,
        "candidate_improves_each_route_by_at_least_m": 0.05,
        "candidate_no_route_regression": True,
        "candidate_macro_improvement_at_least_m": 0.1,
        "candidate_mtv_h_improvement_at_least_m": 0.1,
        "candidate_prediction_domain_coverage": 1.0,
        "candidate_macro_score_max_m": 2.0,
        "candidate_route_score_max_m": 3.0,
        "candidate_mtv_h_p95_max_m": 5.0,
        "candidate_all_finite": True,
        "candidate_over_70_mps_count": 0,
    }
    if any(gates.get(key) != value for key, value in expected_gates.items()):
        raise fail("Phase76 accuracy gates changed")
    policy = freeze.get("read_policy", {})
    expected_policy = {
        "truth_reads_before_freeze": 0,
        "truth_reads_before_manifest": 0,
        "truth_reads_after_manifest": 4,
        "truth_reads_per_route": 1,
        "candidate_artifact_reads": 8,
        "control_artifact_reads": 8,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()) or policy.get("native_or_solver_subprocess") is not False or policy.get("post_truth_tuning") is not False:
        raise fail("Phase76 read contract changed")
    if freeze.get("sealed_phase73_artifact_source", {}).get("route_control_identity") != "for every route, the expected control is read directly from sealed_phase73_artifacts.routes[route].control in the pinned Phase74 freeze; no helper constant or retyped Phase43 table is permitted":
        raise fail("Phase76 control identity source changed")
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


def score_route(freeze: dict[str, Any], freeze74: dict[str, Any], route: str, accounting: dict[str, int]) -> dict[str, Any]:
    candidate_pin, control_pin = _phase74_artifact_pins(freeze74, route)
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
    candidate_rows, candidate_map = P74._parse_submission(candidate_submission, route)
    control_rows, control_map = P74._parse_submission(control_submission, route)
    candidate_diag = P74._validate_summary(candidate_summary, route, True)
    P74._validate_summary(control_summary, route, False)
    if candidate_map.keys() != control_map.keys():
        raise fail(f"candidate/control prediction keys differ: {route}")
    control_identity = _control_identity_from_phase74(freeze74, route, control_pin)
    if not control_identity:
        raise fail(f"control is not the sealed Phase74 route pin: {route}")
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
    telemetry = candidate_diag["miss_mask"]
    telemetry_keys = ("original_adopted_pseudorange_rows", "retained_finite_pc_pseudorange_rows", "dropped_missing_exact_stream_rows", "dropped_out_of_domain_rows", "dropped_nonfinite_correction_rows", "retained_over_original_fraction", "correction_abs_p50_m", "correction_abs_p95_m", "correction_abs_max_m")
    return {
        "truth": {"path": relative(truth_path), "sha256": truth_pin["sha256"], "bytes": len(truth_payload), "rows": len(truth_map), "read_count": 1, "expected_missing_truth_rows": truth_pin["expected_missing_truth_rows"], "expected_missing_key": truth_pin.get("expected_missing_key")},
        "artifact_hashes": {"candidate_submission": candidate_pin["submission_sha256"], "candidate_summary": candidate_pin["summary_sha256"], "control_submission": control_pin["submission_sha256"], "control_summary": control_pin["summary_sha256"]},
        "candidate": candidate_score,
        "control": control_score,
        "improvement_m": improvement,
        "candidate_phase73_telemetry": {key: telemetry[key] for key in telemetry_keys},
        "control_phase43_identity": control_identity,
        "control_identity_source": "Phase74 freeze sealed_phase73_artifacts.routes[route].control",
    }


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    freeze74 = load_json(PHASE74_FREEZE, "Phase74 accuracy freeze")
    if sha256_file(PHASE74_FREEZE) != PHASE74_FREEZE_SHA256:
        raise fail("Phase74 accuracy freeze bytes changed")
    P74.verify_freeze()
    if not MANIFEST.is_file():
        raise fail("Phase76 recovery manifest is missing")
    manifest = load_json(MANIFEST, "Phase76 recovery manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256_file(EVALUATOR) or manifest.get("read_policy", {}).get("truth_reads") != 4 or manifest.get("read_policy", {}).get("native_or_solver_subprocess") is not False:
        raise fail("Phase76 manifest pin/read contract changed")
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase76 output: {output_root}")
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
            "phase": 76,
            "execution_label": "Luna Max",
            "status": "go-phase76-accuracy-gates" if all_passed else "no-go-phase76-accuracy-gates",
            "decision": "candidate may proceed to separately frozen validation" if all_passed else "preserve-phase43-champion; keep-phase73-experimental; no-validation",
            "recovery_only": True,
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR), "single_process": True, "native_or_solver_subprocess": False, "post_truth_tuning": False},
            "phase75_integrity_failure": {"path": relative(PHASE75_FAILURE), "sha256": PHASE75_FAILURE_SHA256, "truth_reads_previous_attempt": 1, "accuracy_observed_previous_attempt": False},
            "phase73_structural_input": {"result_record": {"path": relative(PHASE73_RESULT_RECORD), "sha256": PHASE73_RESULT_RECORD_SHA256}, "candidate_control_source": "Phase73 structural run1 artifacts inherited through the Phase74 freeze; no native rerun"},
            "control_identity_source": "Phase74 freeze sealed_phase73_artifacts.routes[route].control; no PHASE43_CONTROL helper constant",
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
            "fresh_validation": "not run in Phase76; separate freeze required" if all_passed else "not authorized",
            "zero_point_782_claim": "reported only; no promotion or Kaggle action",
        }
        result_path = output_root / "phase76_phase75_accuracy_integrity_recovery_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": "smartphone-r5-phase76-phase75-accuracy-integrity-recovery-output-manifest.v1", "phase": 76, "status": result["status"], "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256_file(result_path), "bytes": result_path.stat().st_size}, "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "native_or_solver_subprocess": 0, "all_gates_passed": all_passed}
        atomic_json(output_root / "phase76_phase75_accuracy_integrity_recovery_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase76-phase75-accuracy-integrity-recovery-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": accounting["truth_reads"], "candidate_artifact_reads": accounting["candidate_artifact_reads"], "control_artifact_reads": accounting["control_artifact_reads"], "routes_completed": sorted(routes), "native_or_solver_subprocess": 0, "post_truth_tuning": False}
        atomic_json(output_root / "phase76_phase75_accuracy_integrity_recovery_failure.json", failure)
        if isinstance(exc, Phase76RecoveryError):
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
    except Phase76RecoveryError as exc:
        print(f"phase76 recovery failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
