#!/usr/bin/env python3
"""Score the sealed Phase80 direct-quality candidate against Phase43 and Phase78 outputs.

Phase82 is an accuracy scorer only.  It consumes immutable submissions and
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
_SPEC = importlib.util.spec_from_file_location("phase76_accuracy_helpers_phase82", PHASE76_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load Phase76 helper: {PHASE76_PATH}")
P76 = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(P76)


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_freeze_v1.json"
FREEZE_SHA256 = "33bbfc4051af20cd3f39fbf8620ea4d277c4acc5dbe5d8d1621452a657eebf6b"
PHASE79_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_freeze_v1.json"
PHASE79_FREEZE_SHA256 = "f57b064405cc8e591f03bd1f76c208b364e8fcc4a06c59106a117911d992b166"
PHASE81_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_freeze_v1.json"
PHASE81_FREEZE_SHA256 = "c648ec5f49f9023239360a70a1d77ee4b546e2b4d6f07cdf076f00694a2aeb6f"
PHASE81_RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_result_v1.json"
PHASE81_RESULT_SHA256 = "f5809f173c3e346aec775ca6dd152de5436eb68ea348d3fe90dffc7f82153b14"
PHASE81_MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_manifest_v1.json"
PHASE81_MANIFEST_SHA256 = "82962ad707f08623f7064d271fdc8ebb7b015668121a8027ba577f40a1ce222a"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_manifest_v1.json"
EVALUATOR = Path(__file__).resolve()
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase82-phase80-direct-quality-accuracy-v1"
OUTPUT_SCHEMA = "smartphone-r5-phase82-phase80-direct-quality-accuracy-result.v1"
OUTPUT_MANIFEST_SCHEMA = "smartphone-r5-phase82-phase80-direct-quality-accuracy-output-manifest.v1"
PHASE43_SEAL_RESULT = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/phase43_structural_seal.json"
PHASE43_SEAL_RESULT_SHA256 = "fdeaf672b015cae99dfdf8351a5e7a92ca2d37bcdb0872c5c3fc5b937416b64d"
PHASE43_SEAL_RESULT_BYTES = 567854
PHASE43_SEAL_MANIFEST = ROOT / "output/smartphone-r5/phase43-native-fallback-seed-quality-anchor-recovery-v1/phase43_structural_seal.manifest.json"
PHASE43_SEAL_MANIFEST_SHA256 = "c116fe122399dafb516b9d76eeab6ea4fb77bcad5aeb9b552f08550d351a26ce"
PHASE43_SEAL_MANIFEST_BYTES = 1075
EARTH_RADIUS_M = 6_371_008.8
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)
TARGET_ROUTE = ROUTES[1]


class Phase82AccuracyError(ValueError):
    """Raised when the immutable Phase82 contract cannot be evaluated."""


def fail(message: str) -> Phase82AccuracyError:
    return Phase82AccuracyError(message)


def reject_forbidden(path: Path | str) -> None:
    token = str(path).lower()
    if token.endswith(".mat") or ".mat/" in token or ".mat\\" in token:
        raise fail(f"MAT path is forbidden: {path}")
    for term in ("validation", "holdout", "kaggle", "token", "device_wls", "svposition", "svelevation"):
        if term in token:
            raise fail(f"forbidden Phase82 path: {path}")


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


def _require_pin(pin: Any, source: str, route: str, require_bytes: bool) -> None:
    if not isinstance(pin, dict):
        raise fail(f"Phase82 artifact pin is not an object: {source}/{route}")
    for key in ("submission_path", "submission_sha256", "summary_path", "summary_sha256"):
        if not isinstance(pin.get(key), str) or not pin[key]:
            raise fail(f"Phase82 {source} pin missing {key}: {route}")
    for key in ("submission_path", "summary_path"):
        path = pin[key]
        if Path(path).is_absolute() or not path.startswith("output/"):
            raise fail(f"Phase82 {source} path is not repository-relative output: {route}")
        _safe_root_path(path, f"{source} {key} {route}")
    for key in ("submission_sha256", "summary_sha256"):
        if len(pin[key]) != 64 or any(char not in "0123456789abcdef" for char in pin[key].lower()):
            raise fail(f"Phase82 {source} hash malformed: {route}/{key}")
    if require_bytes:
        for key in ("submission_bytes", "summary_bytes"):
            value = pin.get(key)
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                raise fail(f"Phase82 {source} byte pin malformed: {route}/{key}")
    rows = pin.get("rows")
    if isinstance(rows, bool) or not isinstance(rows, int) or rows <= 0:
        raise fail(f"Phase82 {source} row pin malformed: {route}")


def _require_route_maps(freeze: dict[str, Any]) -> None:
    cohort = freeze.get("cohort", {})
    if tuple(cohort.get("route_order", ())) != ROUTES or set(cohort.get("truths", {})) != set(ROUTES):
        raise fail("Phase82 route order/truth set changed")
    sources = freeze.get("artifact_sources", {})
    for source in ("phase80_candidate_run1", "phase78_candidate_run1", "phase43_control"):
        block = sources.get(source, {})
        if block.get("read_once_per_route") is not True or set(block.get("routes", {})) != set(ROUTES):
            raise fail(f"Phase82 artifact route map changed: {source}")
        for route in ROUTES:
            _require_pin(block["routes"][route], source, route, source == "phase80_candidate_run1")


def _load_phase43_seal() -> tuple[dict[str, Any], dict[str, Any]]:
    """Read the sealed Phase43 structural metadata once and validate its pin.

    The Phase79 freeze contains the expected SHA/row pins, but its historical
    control path strings omitted the ``candidate/`` component.  The Phase43
    structural seal is the authoritative path resolver: a route is accepted
    only when its recorded run-1 submission/summary path, SHA, byte count, and
    row count are all present and later agree with the Phase79 pins.
    """
    seal_payload = _read_once(PHASE43_SEAL_RESULT, PHASE43_SEAL_RESULT_SHA256, PHASE43_SEAL_RESULT_BYTES, "Phase43 structural seal")
    manifest_payload = _read_once(PHASE43_SEAL_MANIFEST, PHASE43_SEAL_MANIFEST_SHA256, PHASE43_SEAL_MANIFEST_BYTES, "Phase43 structural seal manifest")
    try:
        seal = json.loads(seal_payload.decode("utf-8"))
        seal_manifest = json.loads(manifest_payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise fail("Phase43 structural seal metadata is invalid") from exc
    if not isinstance(seal, dict) or not isinstance(seal_manifest, dict):
        raise fail("Phase43 structural seal metadata is not an object")
    if seal_manifest.get("phase") != 43 or seal_manifest.get("truth_open_count") != 0 or seal_manifest.get("mat_read_or_generated") is not False:
        raise fail("Phase43 structural seal manifest policy changed")
    result_pin = seal_manifest.get("result", {})
    if result_pin.get("path") != relative(PHASE43_SEAL_RESULT) or result_pin.get("sha256") != PHASE43_SEAL_RESULT_SHA256:
        raise fail("Phase43 structural seal manifest result pin changed")
    if not isinstance(seal.get("candidate_runs"), dict):
        raise fail("Phase43 structural seal candidate_runs missing")
    return seal, seal_manifest


def _safe_root_path(relative_path: Any, label: str) -> Path:
    if not isinstance(relative_path, str) or not relative_path or Path(relative_path).is_absolute():
        raise fail(f"{label} path is not a repository-relative path")
    path = (ROOT / relative_path).resolve()
    try:
        path.relative_to(ROOT)
    except ValueError as exc:
        raise fail(f"{label} path escapes repository: {relative_path}") from exc
    return path


def _resolve_phase43_control(freeze: dict[str, Any], seal: dict[str, Any], route: str) -> dict[str, Any]:
    """Resolve one control only from the pinned Phase43 run-1 seal record."""
    try:
        frozen_pin = freeze["artifact_sources"]["phase43_control"]["routes"][route]
        route_seal = seal["candidate_runs"][route]
        run = route_seal["run1"]
        submission = run["submission"]
        summary = run["summary"]
    except (KeyError, TypeError) as exc:
        raise fail(f"Phase43 sealed run-1 control pin missing: {route}") from exc
    if route_seal.get("dataset_id") != route or run.get("candidate") is not True:
        raise fail(f"Phase43 sealed run-1 dataset identity changed: {route}")
    required_submission = ("path", "sha256", "bytes", "rows")
    required_summary = ("path", "sha256", "bytes")
    if any(key not in submission for key in required_submission) or any(key not in summary for key in required_summary):
        raise fail(f"Phase43 sealed run-1 artifact metadata incomplete: {route}")
    if frozen_pin.get("submission_sha256") != submission["sha256"] or frozen_pin.get("submission_bytes") != submission["bytes"] or frozen_pin.get("rows") != submission["rows"] or frozen_pin.get("summary_sha256") != summary["sha256"]:
        raise fail(f"Phase43 sealed run-1 artifact metadata disagrees with Phase79 freeze: {route}")
    submission_path = _safe_root_path(submission["path"], f"Phase43 submission {route}")
    summary_path = _safe_root_path(summary["path"], f"Phase43 summary {route}")
    return {
        "submission_path": relative(submission_path),
        "submission_sha256": submission["sha256"],
        "submission_bytes": int(submission["bytes"]),
        "submission_rows": int(submission["rows"]),
        "summary_path": relative(summary_path),
        "summary_sha256": summary["sha256"],
        "summary_bytes": int(summary["bytes"]),
        "resolver": "Phase43 phase43_structural_seal.json candidate_runs[route].run1",
    }


def _verify_phase81_pins(freeze: dict[str, Any]) -> None:
    """Verify the sealed Phase81 GO result/manifest and candidate identity pins."""
    authority = freeze.get("authority", {})
    expected_docs = {
        "phase81_reclassification_freeze": (PHASE81_FREEZE, PHASE81_FREEZE_SHA256),
        "phase81_reclassification_result": (PHASE81_RESULT, PHASE81_RESULT_SHA256),
        "phase81_reclassification_manifest": (PHASE81_MANIFEST, PHASE81_MANIFEST_SHA256),
    }
    for key, (path, expected_hash) in expected_docs.items():
        pin = authority.get(key)
        if not isinstance(pin, dict) or pin.get("path") != relative(path) or pin.get("sha256") != expected_hash:
            raise fail(f"Phase81 authority pin changed: {key}")
        if sha256_file(path) != expected_hash:
            raise fail(f"Phase81 authority document changed: {key}")

    phase81_freeze = load_json(PHASE81_FREEZE, "Phase81 freeze")
    if phase81_freeze.get("status") != "frozen-before-phase81-sealed-artifact-read":
        raise fail("Phase81 freeze status changed")
    phase81_result = load_json(PHASE81_RESULT, "Phase81 result")
    if phase81_result.get("phase") != 81 or phase81_result.get("status") != "go-phase81-phase80-sealed-artifact-reclassification" or phase81_result.get("truth_free") is not True:
        raise fail("Phase81 result is not the sealed GO result")
    if phase81_result.get("freeze", {}).get("path") != relative(PHASE81_FREEZE) or phase81_result.get("freeze", {}).get("sha256") != PHASE81_FREEZE_SHA256:
        raise fail("Phase81 result freeze pin changed")
    if phase81_result.get("manifest", {}).get("path") != relative(PHASE81_MANIFEST) or phase81_result.get("manifest", {}).get("sha256") != PHASE81_MANIFEST_SHA256:
        raise fail("Phase81 result manifest pin changed")
    phase81_manifest = load_json(PHASE81_MANIFEST, "Phase81 manifest")
    if phase81_manifest.get("phase") != 81 or phase81_manifest.get("freeze", {}).get("sha256") != PHASE81_FREEZE_SHA256 or phase81_manifest.get("routes") != list(ROUTES):
        raise fail("Phase81 manifest freeze/route pin changed")
    if phase81_manifest.get("phase80_v2_failure", {}).get("sha256") != "767ef5f7089e535063ec6cbe4e9dc3e6d061bdab62eb521f794c3fab0b171903":
        raise fail("Phase81 manifest Phase80 failure pin changed")

    pins = phase81_freeze.get("candidate_artifact_pins", {})
    current = freeze.get("artifact_sources", {}).get("phase80_candidate_run1", {}).get("routes", {})
    if tuple(pins) != ROUTES or set(current) != set(ROUTES):
        raise fail("Phase81 candidate route pin set changed")
    for route in ROUTES:
        source_pin = current[route]
        sealed_pin = pins[route].get("candidate_run1", {})
        submission = sealed_pin.get("submission", {})
        summary = sealed_pin.get("summary", {})
        if source_pin.get("submission_path") != submission.get("path") or source_pin.get("submission_sha256") != submission.get("sha256") or source_pin.get("submission_bytes") != submission.get("bytes") or source_pin.get("summary_path") != summary.get("path") or source_pin.get("summary_sha256") != summary.get("sha256") or source_pin.get("summary_bytes") != summary.get("bytes"):
            raise fail(f"Phase80 candidate pin disagrees with Phase81: {route}")


def verify_freeze() -> dict[str, Any]:
    if sha256_file(FREEZE) != FREEZE_SHA256:
        raise fail("Phase82 freeze hash changed")
    freeze = load_json(FREEZE, "Phase82 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase82-phase80-direct-quality-accuracy-freeze.v1" or freeze.get("phase") != 82 or freeze.get("status") != "frozen-before-phase82-truth-read":
        raise fail("Phase82 freeze schema/status changed")
    _verify_phase81_pins(freeze)

    # Phase79 supplies the immutable Phase43/Phase78/truth metadata and the
    # corrected metric contract.  Compare those subtrees exactly, without
    # opening any truth payload.
    if sha256_file(PHASE79_FREEZE) != PHASE79_FREEZE_SHA256:
        raise fail("Phase79 freeze bytes changed")
    phase79 = load_json(PHASE79_FREEZE, "Phase79 freeze metadata")
    for key in ("phase78_structural_result", "phase78_structural_freeze", "phase78_structural_manifest", "phase73_structural_result", "phase74_accuracy_freeze"):
        if freeze.get("authority", {}).get(key) != phase79.get("authority", {}).get(key):
            raise fail(f"Phase79 authority metadata changed: {key}")
        pin = freeze.get("authority", {}).get(key)
        if not isinstance(pin, dict):
            raise fail(f"Phase82 authority pin missing: {key}")
        _pin_hash(_safe_root_path(pin.get("path"), key), pin, key)
    if freeze.get("cohort", {}).get("truths") != phase79.get("cohort", {}).get("truths"):
        raise fail("Phase79 truth metadata changed")
    for source in ("phase78_candidate_run1", "phase43_control"):
        if freeze.get("artifact_sources", {}).get(source) != phase79.get("artifact_sources", {}).get(source):
            raise fail(f"Phase79 artifact metadata changed: {source}")
    _require_route_maps(freeze)

    metric = freeze.get("metric_contract")
    if metric != phase79.get("metric_contract"):
        raise fail("Phase79 corrected DictReader/Haversine metric contract changed")
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
        "candidate_macro_score_strict_max_m": 0.782,
        "candidate_macro_improvement_vs_phase78_min_m": 0.0,
        "candidate_macro_improvement_vs_phase78_strictly_positive": True,
    }
    if any(gates.get(key) != value for key, value in expected_gates.items()):
        raise fail("Phase82 accuracy gates changed")
    policy = freeze.get("read_policy", {})
    expected_policy = {
        "truth_reads_before_freeze": 0,
        "truth_reads_before_manifest": 0,
        "truth_reads_after_manifest": 4,
        "truth_reads_per_route": 1,
        "phase80_candidate_artifact_reads": 8,
        "phase78_candidate_artifact_reads": 8,
        "phase43_control_artifact_reads": 8,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise fail("Phase82 read counts changed")
    if policy.get("single_scorer_process") is not True or any(policy.get(key) is not False for key in ("native_or_solver_subprocess", "post_truth_tuning", "archive_reopen_or_rematerialize", "solver_rerun_after_truth")):
        raise fail("Phase82 forbidden-input policy changed")
    for key in ("native_solver_invocations", "raw_gnss_reads", "raw_imu_reads", "navigation_reads", "mat_reads_or_generated", "kaggle_or_token_access", "kaggle_reads", "token_reads", "validation_holdout_reads"):
        if policy.get(key) != 0:
            raise fail(f"Phase82 forbidden read count changed: {key}")
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


def _artifact(freeze: dict[str, Any], source: str, route: str, accounting: dict[str, int], phase43_seal: dict[str, Any] | None = None) -> tuple[bytes, bytes, dict[str, Any]]:
    if source == "phase43_control":
        if phase43_seal is None:
            raise fail("Phase43 resolver metadata was not loaded")
        pin = _resolve_phase43_control(freeze, phase43_seal, route)
    else:
        pin = freeze["artifact_sources"][source]["routes"][route]
    submission_path = _safe_root_path(pin.get("submission_path"), f"{source} submission {route}")
    summary_path = _safe_root_path(pin.get("summary_path"), f"{source} summary {route}")
    counter = {
        "phase80_candidate_run1": "phase80_candidate_artifact_reads",
        "phase78_candidate_run1": "phase78_candidate_artifact_reads",
        "phase43_control": "phase43_control_artifact_reads",
    }[source]
    accounting[counter] += 1
    submission = _read_once(submission_path, pin["submission_sha256"], pin.get("submission_bytes"), f"{source} submission")
    accounting[counter] += 1
    summary = _read_once(summary_path, pin["summary_sha256"], pin.get("summary_bytes"), f"{source} summary")
    return submission, summary, pin


def score_route(freeze: dict[str, Any], route: str, accounting: dict[str, int]) -> dict[str, Any]:
    phase43_seal = accounting.get("_phase43_seal")
    candidate_submission, candidate_summary_payload, candidate_pin = _artifact(freeze, "phase80_candidate_run1", route, accounting)
    phase78_submission, phase78_summary_payload, phase78_pin = _artifact(freeze, "phase78_candidate_run1", route, accounting)
    control_submission, control_summary_payload, control_pin = _artifact(freeze, "phase43_control", route, accounting, phase43_seal)
    candidate_summary = _validate_summary(candidate_summary_payload, route, "Phase80 candidate")
    _validate_summary(phase78_summary_payload, route, "Phase78 candidate")
    _validate_summary(control_summary_payload, route, "Phase43 control")
    candidate_rows, candidate_map = P76.P74._parse_submission(candidate_submission, route)
    phase78_rows, phase78_map = P76.P74._parse_submission(phase78_submission, route)
    control_rows, control_map = P76.P74._parse_submission(control_submission, route)
    if candidate_map.keys() != control_map.keys() or candidate_map.keys() != phase78_map.keys():
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
    phase78_score = P76._score_prediction(phase78_map, truth_map, expected_missing, route, phase78_rows)
    control_score = P76._score_prediction(control_map, truth_map, expected_missing, route, control_rows)
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
            "phase80_candidate_submission": candidate_pin["submission_sha256"],
            "phase80_candidate_summary": candidate_pin["summary_sha256"],
            "phase78_candidate_submission": phase78_pin["submission_sha256"],
            "phase78_candidate_summary": phase78_pin["summary_sha256"],
            "phase43_control_submission": control_pin["submission_sha256"],
            "phase43_control_summary": control_pin["summary_sha256"],
        },
        "candidate": candidate_score,
        "phase78_candidate": phase78_score,
        "control_phase43": control_score,
        "improvement_vs_phase43_m": control_score["score_m"] - candidate_score["score_m"],
        "improvement_vs_phase78_m": phase78_score["score_m"] - candidate_score["score_m"],
        "candidate_summary_policy": {"dataset_id": candidate_summary.get("dataset_id"), "truth_used": candidate_summary.get("truth_used"), "production_default_changed": candidate_summary.get("production_default_changed")},
    }


def run_score(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    freeze = verify_freeze()
    if not MANIFEST.is_file():
        raise fail("Phase82 accuracy evaluator manifest is missing")
    manifest = load_json(MANIFEST, "Phase82 accuracy evaluator manifest")
    evaluator_sha256 = sha256_file(EVALUATOR)
    if manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256 or manifest.get("evaluator", {}).get("path") != relative(EVALUATOR) or manifest.get("evaluator", {}).get("sha256") != evaluator_sha256:
        raise fail("Phase82 evaluator manifest pin changed")
    read_policy = manifest.get("read_policy", {})
    if read_policy.get("truth_reads") != 4 or read_policy.get("truth_reads_before_manifest") != 0 or read_policy.get("truth_reads_after_manifest") != 4 or read_policy.get("truth_reads_per_route") != 1 or read_policy.get("native_or_solver_subprocess") is not False or read_policy.get("post_truth_tuning") is not False:
        raise fail("Phase82 manifest read contract changed")
    if manifest.get("phase43_baseline_resolver", {}).get("result_sha256") != PHASE43_SEAL_RESULT_SHA256 or manifest.get("phase43_baseline_resolver", {}).get("manifest_sha256") != PHASE43_SEAL_MANIFEST_SHA256:
        raise fail("Phase82 Phase43 baseline resolver pin changed")
    phase43_seal, _phase43_seal_manifest = _load_phase43_seal()
    output_root = output_root.resolve()
    reject_forbidden(output_root)
    if output_root.exists() and any(output_root.iterdir()):
        raise fail(f"refusing to overwrite nonempty Phase82 output: {output_root}")
    output_root.mkdir(parents=True, exist_ok=True)
    accounting = {
        "phase80_candidate_artifact_reads": 0,
        "phase78_candidate_artifact_reads": 0,
        "phase43_control_artifact_reads": 0,
        "truth_reads": 0,
    }
    accounting["_phase43_seal"] = phase43_seal
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
        phase78_scores = [routes[route]["phase78_candidate"]["score_m"] for route in ROUTES]
        candidate_macro = sum(candidate_scores) / len(candidate_scores)
        control_macro = sum(control_scores) / len(control_scores)
        phase78_macro = sum(phase78_scores) / len(phase78_scores)
        macro_improvement_vs_phase43 = control_macro - candidate_macro
        macro_improvement_vs_phase78 = phase78_macro - candidate_macro
        phase78_gate_threshold = float(freeze["accuracy_gates"]["candidate_macro_improvement_vs_phase78_min_m"])
        strict_macro_max = float(freeze["accuracy_gates"]["candidate_macro_score_strict_max_m"])
        gates = {
            "all_four_routes": len(routes) == 4,
            "candidate_each_route_improves_0_05m": all(routes[route]["improvement_vs_phase43_m"] >= 0.05 for route in ROUTES),
            "candidate_no_route_regression": all(routes[route]["improvement_vs_phase43_m"] >= 0.0 for route in ROUTES),
            "candidate_macro_improvement_at_least_0_10m": macro_improvement_vs_phase43 >= 0.1,
            "candidate_mtv_h_improvement_at_least_0_10m": routes[TARGET_ROUTE]["improvement_vs_phase43_m"] >= 0.1,
            "candidate_macro_score_at_most_2m": candidate_macro <= 2.0,
            "candidate_each_route_score_at_most_3m": all(routes[route]["candidate"]["score_m"] <= 3.0 for route in ROUTES),
            "candidate_mtv_h_p95_at_most_5m": routes[TARGET_ROUTE]["candidate"]["p95_m"] <= 5.0,
            "candidate_prediction_domain_coverage_exact": all(routes[route]["candidate"]["prediction_domain_coverage"] == 1.0 for route in ROUTES),
            "candidate_over_70_mps_count_zero": all(routes[route]["candidate"]["over_70_mps_count"] == 0 for route in ROUTES),
            "all_finite": all(routes[route]["candidate"]["finite"] and routes[route]["control_phase43"]["finite"] and routes[route]["phase78_candidate"]["finite"] for route in ROUTES),
            "candidate_macro_score_at_most_0_782m": candidate_macro <= strict_macro_max,
            "candidate_macro_improvement_vs_phase78_strictly_positive": macro_improvement_vs_phase78 > phase78_gate_threshold,
        }
        all_passed = all(gates.values())
        result = {
            "schema_version": OUTPUT_SCHEMA,
            "phase": 82,
            "execution_label": "Luna Max",
            "status": "go-phase82-accuracy-gates" if all_passed else "no-go-phase82-accuracy-gates",
            "decision": "request separately frozen validation phase" if all_passed else "preserve-phase43-champion; preserve-phase80-and-phase81-experiments; no-validation",
            "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256},
            "manifest": {"path": relative(MANIFEST), "sha256": sha256_file(MANIFEST)},
            "evaluator": {"path": relative(EVALUATOR), "sha256": evaluator_sha256, "single_process": True, "native_or_solver_subprocess": False, "post_truth_tuning": False},
            "metric_contract": freeze["metric_contract"],
            "comparison": {"candidate": "Phase80 v2 direct/no-PDC observable-quality candidate run1", "control": "exact Phase43 control route pins", "phase78": "Phase78 sealed candidate run1; macro improvement must be strictly positive"},
            "routes": routes,
            "aggregate": {"candidate_macro_score_m": candidate_macro, "control_phase43_macro_score_m": control_macro, "phase78_candidate_macro_score_m": phase78_macro, "macro_improvement_vs_phase43_m": macro_improvement_vs_phase43, "macro_improvement_vs_phase78_m": macro_improvement_vs_phase78, "candidate_route_count": len(candidate_scores), "control_route_count": len(control_scores), "phase78_route_count": len(phase78_scores)},
            "accuracy_gates": gates | {"all_passed": all_passed},
            "strict_0_782": {"target_score_m": strict_macro_max, "candidate_macro_score_m": candidate_macro, "met": candidate_macro <= strict_macro_max, "and_gate": True, "validation_or_kaggle": False},
            "phase43_baseline_resolver": {"result_path": relative(PHASE43_SEAL_RESULT), "result_sha256": PHASE43_SEAL_RESULT_SHA256, "result_bytes": PHASE43_SEAL_RESULT_BYTES, "manifest_path": relative(PHASE43_SEAL_MANIFEST), "manifest_sha256": PHASE43_SEAL_MANIFEST_SHA256, "manifest_bytes": PHASE43_SEAL_MANIFEST_BYTES, "route_source": "candidate_runs[route].run1.submission/summary", "metadata_reads": 2, "summary": "Phase43 seal is the deterministic path authority; every recorded SHA/bytes/rows must agree with the Phase82 freeze before a baseline artifact is opened"},
            "read_accounting": {"single_scorer_process": True, "phase80_candidate_artifact_reads": accounting["phase80_candidate_artifact_reads"], "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase43_seal_metadata_reads": 2, "truth_reads": accounting["truth_reads"], "truth_reads_per_route": 1, "raw_gnss_reads": 0, "raw_imu_reads": 0, "navigation_reads": 0, "native_or_solver_subprocess": 0, "validation_holdout_reads": 0, "mat_reads_or_generated": 0, "device_wls_or_precomputed_coordinates": 0, "kaggle_or_token_access": 0, "kaggle_reads": 0, "token_reads": 0, "archive_reopen_or_rematerialize": False, "post_truth_tuning": False},
            "phase43_champion_preserved": True,
            "phase78_experimental_preserved": True,
            "phase80_experimental_preserved": True,
            "phase81_structural_go_preserved": True,
            "fresh_validation": "not run in Phase82; separate freeze required" if all_passed else "not authorized",
            "zero_point_782_claim": "strict AND gate evaluated; no promotion or Kaggle action",
        }
        result_path = output_root / "phase82_phase80_direct_quality_accuracy_result.json"
        atomic_json(result_path, result)
        output_manifest = {"schema_version": OUTPUT_MANIFEST_SCHEMA, "phase": 82, "status": result["status"], "freeze": {"path": relative(FREEZE), "sha256": FREEZE_SHA256}, "evaluator": {"path": relative(EVALUATOR), "sha256": evaluator_sha256}, "result": {"path": relative(result_path), "sha256": sha256_file(result_path), "bytes": result_path.stat().st_size}, "truth_reads": accounting["truth_reads"], "truth_reads_before_manifest": 0, "truth_reads_after_manifest": accounting["truth_reads"], "truth_reads_per_route": 1, "phase80_candidate_artifact_reads": accounting["phase80_candidate_artifact_reads"], "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase43_seal_metadata_reads": 2, "native_or_solver_subprocess": 0, "raw_gnss_reads": 0, "raw_imu_reads": 0, "mat_reads_or_generated": 0, "kaggle_or_token_access": 0, "validation_holdout_reads": 0, "post_truth_tuning": False, "all_gates_passed": all_passed}
        atomic_json(output_root / "phase82_phase80_direct_quality_accuracy_manifest.json", output_manifest)
        return result
    except Exception as exc:
        failure = {"schema_version": "smartphone-r5-phase82-phase80-direct-quality-accuracy-failure.v1", "status": "fail-closed", "exception_type": type(exc).__name__, "error": str(exc), "truth_reads": accounting["truth_reads"], "truth_reads_before_manifest": 0, "truth_reads_after_manifest": accounting["truth_reads"], "truth_reads_per_route": 1, "phase80_candidate_artifact_reads": accounting["phase80_candidate_artifact_reads"], "phase78_candidate_artifact_reads": accounting["phase78_candidate_artifact_reads"], "phase43_control_artifact_reads": accounting["phase43_control_artifact_reads"], "phase43_seal_metadata_reads": 2, "routes_completed": sorted(routes), "native_or_solver_subprocess": 0, "raw_gnss_reads": 0, "raw_imu_reads": 0, "mat_reads_or_generated": 0, "kaggle_or_token_access": 0, "validation_holdout_reads": 0, "post_truth_tuning": False}
        atomic_json(output_root / "phase82_phase80_direct_quality_accuracy_failure.json", failure)
        if isinstance(exc, Phase82AccuracyError):
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
    except Phase82AccuracyError as exc:
        print(f"phase82 accuracy failure: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
