#!/usr/bin/env python3
"""Score the sealed Phase78 candidate against Phase43 and Phase73 outputs.

Phase79 is an accuracy scorer only.  It consumes immutable submissions and
summaries from the already completed structural phases and the four declared
Phase44 development truths.  It does not launch a native process, read raw
GNSS/IMU/navigation inputs, reopen an archive, or perform any post-score
tuning.  The corrected Phase76 CSV ``DictReader`` truth contract is reused:
required fields are selected by name, optional columns are accepted, and a
present ``phone`` column must match the declared route.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import tempfile
from typing import Any


ROOT = Path(__file__).resolve().parents[3]
PHASE76_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
_SPEC = importlib.util.spec_from_file_location("phase76_accuracy_helpers_phase79", PHASE76_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase76 helper: {PHASE76_PATH}")
P76 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P76)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_freeze_v1.json"
FREEZE_SHA256 = "f57b064405cc8e591f03bd1f76c208b364e8fcc4a06c59106a117911d992b166"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase79-phase78-signal-bias-accuracy-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase79-phase78-signal-bias-accuracy-result.v1"
OUTPUT_MANIFEST_SCHEMA = "smartphone-r5-phase79-phase78-signal-bias-accuracy-output-manifest.v1"
EARTH_RADIUS_M = 6_371_008.8
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET_ROUTE = ROUTES[1]


class Phase79AccuracyError(ValueError):
    """Raised when the immutable Phase79 contract cannot be evaluated."""


def fail(message: str) -> Phase79AccuracyError:
    return Phase79AccuracyError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("validation", "holdout", "kaggle", "token", "device_wls", "svposition", "svelevation"):
        if term in token:
            raise fail(f"forbidden Phase79 path: {path}")


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
    atomic_write(path, (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode())


def relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _finite(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))


def _all_json_numbers_finite(value: Any) -> bool:
    if isinstance(value, float):
        return math.isfinite(value)
    if isinstance(value, dict):
        return all(_all_json_numbers_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_all_json_numbers_finite(item) for item in value)
    return True


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


def _pin_hash(path: Path, pin: dict[str, Any], label: str) -> None:
    expected = pin.get("sha256")
    if not isinstance(expected, str) or len(expected) != 64 or sha256_file(path) != expected:
        raise fail(f"{label} hash pin changed: {path}")


def _require_route_maps(freeze: dict[str, Any]) -> None:
    cohort = freeze.get("cohort", {})
    if tuple(cohort.get("route_order", ())) != ROUTES or set(cohort.get("truths", {})) != set(ROUTES):
        raise fail("Phase79 route order/truth set changed")
    sources = freeze.get("artifact_sources", {})
    for source in ("phase78_candidate_run1", "phase43_control", "phase73_no_bias"):
        block = sources.get(source, {})
        if block.get("read_once_per_route") is not True or set(block.get("routes", {})) != set(ROUTES):
            raise fail(f"Phase79 artifact route map changed: {source}")
        for route in ROUTES:
            pin = block["routes"][route]
            for key in ("submission_path", "submission_sha256", "summary_path", "summary_sha256"):
                if not isinstance(pin.get(key), str) or not pin[key]:
                    raise fail(f"Phase79 {source} pin missing {key}: {route}")


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase79 freeze hash changed")
    freeze = load_json(FREEZE, "Phase79 freeze")
    if freeze.get("status") != "frozen-before-phase79-truth-read":
        raise fail("Phase79 freeze status changed")
    authority = freeze.get("authority", {})
    for key in ("phase78_structural_result", "phase78_structural_freeze", "phase78_structural_manifest", "phase73_structural_result", "phase74_accuracy_freeze"):
        pin = authority.get(key)
        if not isinstance(pin, dict) or not isinstance(pin.get("path"), str) or not isinstance(pin.get("sha256"), str):
            raise fail(f"Phase79 authority pin missing: {key}")
        _pin_hash(ROOT / pin["path"], pin, key)
    _require_route_maps(freeze)
    metric = freeze.get("metric_contract", {})
    expected_metric = {
        "key": "(phone, UnixTimeMillis)",
        "required_truth_fields": ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"],
        "prediction_domain_coverage": "matched prediction keys / prediction keys; required exactly 1.0",
        "truth_row_coverage": "informational only; expected leading warm-up exclusions are pinned per route",
        "duplicates": "duplicate prediction or truth keys fail closed",
        "extras": "prediction keys absent from truth fail closed",
        "distance": "spherical Haversine per row",
        "earth_radius_m": EARTH_RADIUS_M,
        "percentile": "linear interpolation at rank (n - 1) * q",
        "score": "(P50 + P95) / 2",
    }
    if any(metric.get(key) != value for key, value in expected_metric.items()):
        raise fail("Phase79 metric contract changed")
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
        raise fail("Phase79 accuracy gates changed")
    policy = freeze.get("read_policy", {})
    expected_policy = {
        "truth_reads_before_freeze": 0,
        "truth_reads_before_manifest": 0,
        "truth_reads_after_manifest": 4,
        "truth_reads_per_route": 1,
        "phase78_candidate_artifact_reads": 8,
        "phase43_control_artifact_reads": 8,
        "phase73_artifact_reads": 8,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise fail("Phase79 read counts changed")
    if policy.get("single_scorer_process") is not True or any(policy.get(key) is not False for key in ("native_or_solver_subprocess", "post_truth_tuning", "archive_reopen_or_rematerialize")) or policy.get("phase51_historical_reference_is_metadata_only") is not True or any(policy.get(key) != 0 for key in ("validation_holdout_reads", "mat_reads_or_generated", "kaggle_token_reads")):
        raise fail("Phase79 forbidden-input policy changed")
    for route in ROUTES:
        truth = freeze["cohort"]["truths"][route]
        if truth.get("expected_missing_truth_rows") not in (0, 1):
            raise fail(f"unexpected warm-up exclusion declaration: {route}")
        missing = truth.get("expected_missing_key")
        if truth["expected_missing_truth_rows"] == 0 and missing is not None:
            raise fail(f"unexpected missing truth key: {route}")
        if truth["expected_missing_truth_rows"] == 1 and (not isinstance(missing, list) or len(missing) != 2 or missing[0] != route):
            raise fail(f"warm-up key declaration changed: {route}")
    return freeze


def _validate_summary(payload: bytes, route: str, label: str) -> dict[str, Any]:
    try:
        summary = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail(f"invalid {label} summary: {route}") from exc
    if not isinstance(summary, dict) or summary.get("dataset_id") != route:
        raise fail(f"{label} summary dataset mismatch: {route}")
    if summary.get("truth_used") is not False or summary.get("production_default_changed") is not False:
        raise fail(f"{label} summary policy mismatch: {route}")
    if not _all_json_numbers_finite(summary):
        raise fail(f"{label} summary has nonfinite telemetry: {route}")
    return summary


def _artifact(freeze: dict[str, Any], source: str, route: str, accounting: dict[str, int]) -> tuple[bytes, bytes, dict[str, Any]]:
    pin = freeze["artifact_sources"][source]["routes"][route]
    submission_path = ROOT / pin["submission_path"]
    summary_path = ROOT / pin["summary_path"]
    counter = {
        "phase78_candidate_run1": "phase78_candidate_artifact_reads",
        "phase43_control": "phase43_control_artifact_reads",
        "phase73_no_bias": "phase73_artifact_reads",
    }[source]
    accounting[counter] += 1
    submission = _read_once(submission_path, pin["submission_sha256"], pin.get("submission_bytes"), f"{source} submission")
    accounting[counter] += 1
    summary = _read_once(summary_path, pin["summary_sha256"], pin.get("summary_bytes"), f"{source} summary")
    return submission, summary, pin


def score_route(freeze: dict[str, Any], route: str, accounting: dict[str, int]) -> dict[str, Any]:
    candidate_submission, candidate_summary_payload, candidate_pin = _artifact(freeze, "phase78_candidate_run1", route, accounting)
    control_submission, control_summary_payload, control_pin = _artifact(freeze, "phase43_control", route, accounting)
    phase73_submission, phase73_summary_payload, phase73_pin = _artifact(freeze, "phase73_no_bias", route, accounting)
    candidate_summary = _validate_summary(candidate_summary_payload, route, "Phase78 candidate")
    _validate_summary(control_summary_payload, route, "Phase43 control")
    _validate_summary(phase73_summary_payload, route, "Phase73 candidate")
    candidate_rows, candidate_map = P76.P74._parse_submission(candidate_submission, route)
    control_rows, control_map = P76.P74._parse_submission(control_submission, route)
    phase73_rows, phase73_map = P76.P74._parse_submission(phase73_submission, route)
    if candidate_map.keys() != control_map.keys() or candidate_map.keys() != phase73_map.keys():
        raise fail(f"prediction domain differs among sealed artifacts: {route}")
    truth_pin = freeze["cohort"]["truths"][route]
    truth_path = ROOT / truth_pin["path"]
    accounting["truth_reads"] += 1
    truth_payload = _read_once(truth_path, truth_pin["sha256"], int(truth_pin["bytes"]), "development truth")
    # Phase76's corrected DictReader is intentionally the only truth parser.
    truth_map = P76._parse_truth_dictreader(truth_payload, route)
    if len(truth_map) != int(truth_pin["rows"]):
        raise fail(f"truth row count mismatch: {route}")
    expected_missing = truth_pin.get("expected_missing_key")
    candidate_score = P76._score_prediction(candidate_map, truth_map, expected_missing, route, candidate_rows)
    control_score = P76._score_prediction(control_map, truth_map, expected_missing, route, control_rows)
    phase73_score = P76._score_prediction(phase73_map, truth_map, expected_missing, route, phase73_rows)
    return {
        "truth": {
            "path": relative(truth_path),
            "sha256": truth_pin["sha256"],
            "bytes": len(truth_payload),
            "rows": len(truth_map),
            "read_count": 1,
            "expected_missing_truth_rows": truth_pin["expected_missing_truth_rows"],
            "expected_missing_key": expected_missing,
            "truth_row_coverage_informational": candidate_score["truth_row_coverage"],
        },
        "artifact_hashes": {
            "phase78_candidate_submission": candidate_pin["submission_sha256"],
            "phase78_candidate_summary": candidate_pin["summary_sha256"],
            "phase43_control_submission": control_pin["submission_sha256"],
            "phase43_control_summary": control_pin["summary_sha256"],
            "phase73_submission": phase73_pin["submission_sha256"],
            "phase73_summary": phase73_pin["summary_sha256"],
        },
        "candidate": candidate_score,
        "control_phase43": control_score,
        "phase73_no_bias": phase73_score,
        "improvement_vs_phase43_m": control_score["score_m"] - candidate_score["score_m"],
        "delta_vs_phase73_m": phase73_score["score_m"] - candidate_score["score_m"],
        "candidate_summary_policy": {"dataset_id": candidate_summary.get("dataset_id"), "truth_used": candidate_summary.get("truth_used"), "production_default_changed": candidate_summary.get("production_default_changed")},
    }


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    if not MANIFEST.is_file():
        raise fail("Phase79 accuracy evaluator manifest is missing")
    manifest = load_json(MANIFEST, "Phase79 accuracy evaluator manifest")
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != sha256_file(EVALUATOR):
        raise fail("Phase79 evaluator manifest pin changed")
    if manifest.get("read_policy", {}).get("truth_reads") != 4 or manifest.get("read_policy", {}).get("truth_reads_per_route") != 1 or manifest.get("read_policy", {}).get("native_or_solver_subprocess") is not False:
        raise fail("Phase79 manifest read contract changed")
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase79 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    accounting = {
        "phase78_candidate_artifact_reads": 0,
        "phase43_control_artifact_reads": 0,
        "phase73_artifact_reads": 0,
        "truth_reads": 0,
    }
    routes: dict[str, Any] = {}
    try:
        for route in ROUTES:
            report = score_route(freeze, route, accounting)
            candidate = report["candidate"]
            improvement = report["improvement_vs_phase43_m"]
            route_gate = {
                "candidate_improvement_at_least_0_05m": improvement >= 0.05,
                "candidate_no_route_regression": improvement >= 0.0,
                "candidate_prediction_domain_coverage_exact": candidate["prediction_domain_coverage"] == 1.0,
                "candidate_finite": candidate["finite"],
                "candidate_over_70_mps_count_zero": candidate["over_70_mps_count"] == 0,
                "candidate_route_score_at_most_3m": candidate["score_m"] <= 3.0,
            }
            report["gate"] = {"passed": all(route_gate.values()), "checks": route_gate, "failures": [key for key, value in route_gate.items() if not value]}
            routes[route] = report
        candidate_scores = [routes[route]["candidate"]["score_m"] for route in ROUTES]
        control_scores = [routes[route]["control_phase43"]["score_m"] for route in ROUTES]
        phase73_scores = [routes[route]["phase73_no_bias"]["score_m"] for route in ROUTES]
        candidate_macro = sum(candidate_scores) / len(candidate_scores)
        control_macro = sum(control_scores) / len(control_scores)
        phase73_macro = sum(phase73_scores) / len(phase73_scores)
        macro_improvement = control_macro - candidate_macro
        gates = {
            "all_four_routes": len(routes) == 4,
            "candidate_each_route_improves_0_05m": all(routes[route]["gate"]["passed"] for route in ROUTES),
            "candidate_macro_improvement_at_least_0_10m": macro_improvement >= 0.1,
            "candidate_mtv_h_improvement_at_least_0_10m": routes[TARGET_ROUTE]["improvement_vs_phase43_m"] >= 0.1,
            "candidate_macro_score_at_most_2m": candidate_macro <= 2.0,
            "candidate_each_route_score_at_most_3m": all(routes[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES),
            "candidate_mtv_h_p95_at_most_5m": routes[TARGET_ROUTE]["candidate"]["p95_m"] <= 5.0,
            "candidate_prediction_domain_coverage_exact": all(routes[route]["candidate"]["prediction_domain_coverage"] == 1.0 for route in ROUTES),
            "candidate_over_70_mps_count_zero": all(routes[route]["candidate"]["over_70_mps_count"] == 0 for route in ROUTES),
            "all_finite": all(routes[route]["candidate"]["finite"] and routes[route]["control_phase43"]["finite"] and routes[route]["phase73_no_bias"]["finite"] for route in ROUTES),
        }
        all_passed = all(gates.values())
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 79,
            "execution_label": "Luna Max",
            "status": "go-phase79-accuracy-gates" if all_passed else "no-go-phase79-accuracy-gates",
            "decision": "request separately frozen validation phase" if all_passed else "preserve-phase43-champion; preserve-phase51-experimental; preserve-phase73-and-phase78-experiments; no-validation",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR), "single_process": True, "native_or_solver_subprocess": False, "post_truth_tuning": False},
            "metric_contract": freeze["metric_contract"],
            "comparison": {"candidate": "Phase78 holistic multi-frequency plus static signal-bias candidate run1", "control": "exact Phase43 control route pins", "phase73": "Phase73 no-bias candidate run1; reported separately and not a gate"},
            "routes": routes,
            "aggregate": {"candidate_macro_score_m": candidate_macro, "control_phase43_macro_score_m": control_macro, "phase73_no_bias_macro_score_m": phase73_macro, "macro_improvement_vs_phase43_m": macro_improvement, "candidate_delta_vs_phase73_m": phase73_macro - candidate_macro, "candidate_route_count": len(candidate_scores), "control_route_count": len(control_scores), "phase73_route_count": len(phase73_scores)},
            "accuracy_gates": gates | {"all_passed": all_passed},
            "stretch_0_782": {"target_score_m": 0.782, "candidate_macro_score_m": candidate_macro, "met": candidate_macro <= 0.782, "report_only": True, "validation_or_kaggle": False},
            "read_accounting": {"single_scorer_process": True, "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase73_artifact_reads": accounting["phase73_artifact_reads"], "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "raw_gnss_reads": 0, "raw_imu_reads": 0, "navigation_reads": 0, "native_or_solver_subprocess": 0, "validation_holdout_reads": 0, "mat_reads_or_generated": 0, "device_wls_or_precomputed_coordinates": 0, "kaggle_or_token_access": 0, "archive_reopen_or_rematerialize": False, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase73_experimental_preserved": True,
            "phase78_experimental_preserved": True,
            "fresh_validation": "not run in Phase79; separate freeze required" if all_passed else "not authorized",
            "zero_point_782_claim": "reported only; no promotion or Kaggle action",
        }
        result_path = output_root / "phase79_phase78_signal_bias_accuracy_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": OUTPUT_MANIFEST_SCHEMA, "phase": 79, "status": result["status"], "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": sha256_file(EVALUATOR)}, "result": {"path": relative(result_path), "sha256": sha256_file(result_path), "bytes": result_path.stat().st_size}, "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase73_artifact_reads": accounting["phase73_artifact_reads"], "native_or_solver_subprocess": 0, "all_gates_passed": all_passed}
        atomic_json(output_root / "phase79_phase78_signal_bias_accuracy_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase79-phase78-signal-bias-accuracy-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase73_artifact_reads": accounting["phase73_artifact_reads"], "routes_completed": sorted(routes), "native_or_solver_subprocess": 0, "post_truth_tuning": False}
        atomic_json(output_root / "phase79_phase78_signal_bias_accuracy_failure.json", failure)
        if isinstance(exc, Phase79AccuracyError):
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
    except Phase79AccuracyError as exc:
        print(f"phase79 accuracy failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
