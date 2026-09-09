#!/usr/bin/env python3
"""Score the frozen Phase29 no-bridge lanes in one development process.

The native candidate and Phase25 control are already sealed artifacts.  This
command never runs the solver.  ``materialize-truth`` extracts only the three
allowlisted development ``ground_truth.csv`` members after the Phase29 freeze;
``score`` verifies all prediction hashes first, then parses each truth file
exactly once and scores control and candidate on their common truth-matched key
set.  Validation, holdout, MAT, and leaderboard inputs are rejected.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any
import zipfile

COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"

if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_kaggle as kaggle  # noqa: E402


FREEZE_SCHEMA = "smartphone-r5-phase29-native-fgo-no-bridge-train-evaluation-freeze.v1"
MATERIALIZATION_SCHEMA = "smartphone-r5-phase29-train-truth-materialization.v1"
RESULT_SCHEMA = "smartphone-r5-phase29-native-fgo-no-bridge-train-evaluation.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase29-native-fgo-no-bridge-train-evaluation-manifest.v1"
PHASE24_SCHEMA = "smartphone-r5-phase24-route-group-split-freeze.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase29_native_fgo_no_bridge_train_eval_freeze_v1.json"
ERRATUM = ROOT / "docs/use_cases/records/smartphone_r5_phase29_native_fgo_no_bridge_train_eval_freeze_erratum_v1.json"
PHASE24_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase24_route_group_split_freeze_v1.json"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase29-no-bridge-train-eval-v1"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
PHASE24_SHA256 = "a20fb65524648fab76c5494d2f28b1846e48fae821296bbdfc212cf6519e4a62"
ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-07-27-19-49-us-ca-mtv-b/pixel4",
    "2021-07-14-20-50-us-ca-mtv-e/sm-g988b",
)
FRESH_VALIDATION = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
FUTURE_HOLDOUT = "2023-05-16-19-54-us-ca-mtv-xe1/pixel5"
DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in kaggle.DISTANCE_VARIANT_IDS
    for percentile in kaggle.PERCENTILE_VARIANT_IDS
)
MAX_CONTINUITY_SPEED_MPS = 70.0
TOLERANCE = 1e-12


class Phase29Error(ValueError):
    """Raised when the frozen Phase29 contract cannot be proven."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase29Error(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise Phase29Error(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase29Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase29Error(f"{label} must be an object: {path}")
    return payload


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
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


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def safe_id(dataset_id: str) -> str:
    return dataset_id.replace("/", "__")


def _verify_freeze(path: Path) -> dict[str, Any]:
    freeze = load_json(path, "Phase29 freeze record")
    if freeze.get("schema_version") != FREEZE_SCHEMA:
        raise Phase29Error("Phase29 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-train-truth-materialization-or-open":
        raise Phase29Error("Phase29 freeze is not pre-truth")
    archive = freeze.get("archive")
    if not isinstance(archive, dict) or archive.get("sha256") != ARCHIVE_SHA256:
        raise Phase29Error("Phase29 archive pin mismatch")
    identities = freeze.get("identities")
    if not isinstance(identities, dict):
        raise Phase29Error("Phase29 identities are missing")
    if tuple(identities.get("development_train", ())) != ROUTES:
        raise Phase29Error("Phase29 train route order mismatch")
    validation = identities.get("fresh_validation", {})
    holdout = identities.get("future_holdout", {})
    if validation.get("id") != FRESH_VALIDATION or holdout.get("id") != FUTURE_HOLDOUT:
        raise Phase29Error("Phase29 validation/holdout identity mismatch")
    if validation.get("remains_sealed") is not True or holdout.get("remains_sealed") is not True:
        raise Phase29Error("Phase29 validation/holdout seal is open")
    inference = freeze.get("inference_contract")
    if not isinstance(inference, dict):
        raise Phase29Error("Phase29 inference contract is missing")
    if inference.get("candidate_flags") != [
        "--native-pdc-imu-tdcp-no-bridge",
        "--android-raw-clock-only",
        "--android-utc-wall-clock-fallback",
        "--android-raw-utc-keys",
    ]:
        raise Phase29Error("Phase29 candidate flags changed")
    if inference.get("control_id") != "phase25_raw_clock_no_bridge_control":
        raise Phase29Error("Phase29 control identity changed")
    if inference.get("candidate_or_control_change_authorized") is not False:
        raise Phase29Error("Phase29 permits candidate/control changes")
    if inference.get("solver_rerun_authorized") is not False:
        raise Phase29Error("Phase29 permits solver rerun")
    forbidden = {str(value).lower() for value in inference.get("forbidden_inputs", ())}
    if not any(".mat" in value for value in forbidden):
        raise Phase29Error("Phase29 lacks MAT prohibition")
    policy = freeze.get("authorization")
    if not isinstance(policy, dict) or any(
        policy.get(key) is not expected
        for key, expected in (
            ("truth_open_count_before_freeze", 0),
            ("truth_materialized_before_freeze", False),
            ("fresh_validation_truth_open_count_before_freeze", 0),
            ("future_holdout_truth_open_count_before_freeze", 0),
            ("mat_read_or_generated", False),
            ("token_access", False),
            ("kaggle_or_external_mutation", False),
            ("no_post_holdout_or_score_tuning", True),
        )
    ):
        raise Phase29Error("Phase29 authorization is not closed")
    routes = freeze.get("sealed_route_artifacts")
    if not isinstance(routes, dict) or tuple(routes) != ROUTES:
        raise Phase29Error("Phase29 sealed route order mismatch")
    # The immutable freeze contains one path-only typo in the Samsung control
    # summary.  Apply the separately committed orchestration erratum only in
    # memory, after verifying the freeze hash and before any truth operation.
    # No algorithm, parameter, or artifact hash is allowed to be overridden.
    erratum = load_json(ERRATUM, "Phase29 freeze erratum")
    if erratum.get("schema_version") != "smartphone-r5-phase29-native-fgo-no-bridge-train-evaluation-freeze-erratum.v1" or erratum.get("status") != "sealed-pre-truth-orchestration-erratum":
        raise Phase29Error("Phase29 freeze erratum schema/status mismatch")
    erratum_freeze = erratum.get("freeze_record")
    if not isinstance(erratum_freeze, dict) or erratum_freeze.get("path") != str(path.relative_to(ROOT)) or erratum_freeze.get("sha256") != sha256(path):
        raise Phase29Error("Phase29 erratum does not pin the immutable freeze")
    correction = erratum.get("correction")
    if not isinstance(correction, dict) or correction.get("algorithm_or_parameter_change") is not False or correction.get("candidate_artifact_change") is not False or correction.get("control_artifact_change") is not False:
        raise Phase29Error("Phase29 erratum is not path-only")
    expected_field = "sealed_route_artifacts.2021-07-14-20-50-us-ca-mtv-e/sm-g988b.control.summary"
    if correction.get("field") != expected_field:
        raise Phase29Error("Phase29 erratum field mismatch")
    if routes[ROUTES[2]]["control"].get("summary") != correction.get("recorded_value"):
        raise Phase29Error("Phase29 freeze erratum recorded path mismatch")
    freeze = json.loads(json.dumps(freeze))
    freeze["sealed_route_artifacts"][ROUTES[2]]["control"]["summary"] = correction["corrected_value"]
    routes = freeze["sealed_route_artifacts"]
    if sha256(ROOT / correction["corrected_value"]) != correction.get("corrected_summary_sha256"):
        raise Phase29Error("Phase29 corrected control summary hash mismatch")
    for dataset_id in ROUTES:
        row = routes.get(dataset_id)
        if not isinstance(row, dict) or not isinstance(row.get("control"), dict) or not isinstance(row.get("candidate"), dict):
            raise Phase29Error(f"Phase29 sealed artifacts missing: {dataset_id}")
        for lane in ("control", "candidate"):
            artifact = row[lane]
            for key in ("submission", "summary", "submission_sha256", "summary_sha256"):
                if not isinstance(artifact.get(key), str) or not artifact[key]:
                    raise Phase29Error(f"Phase29 {lane} artifact field missing: {dataset_id}/{key}")
            for key in ("submission", "summary"):
                artifact_path = ROOT / artifact[key]
                if artifact_path.suffix.lower() == ".mat":
                    raise Phase29Error("MAT artifact is forbidden")
                expected_hash = artifact[f"{key}_sha256"]
                if sha256(artifact_path) != expected_hash:
                    raise Phase29Error(f"sealed {lane} artifact hash mismatch: {dataset_id}/{key}")
    return freeze


def _phase24_metadata() -> dict[str, Any]:
    if sha256(PHASE24_FREEZE) != PHASE24_SHA256:
        raise Phase29Error("Phase24 central metadata record hash mismatch")
    payload = load_json(PHASE24_FREEZE, "Phase24 split freeze")
    if payload.get("schema_version") != PHASE24_SCHEMA:
        raise Phase29Error("Phase24 split freeze schema mismatch")
    metadata = payload.get("selected_central_directory_metadata")
    if not isinstance(metadata, dict):
        raise Phase29Error("Phase24 central metadata is missing")
    return metadata


def materialize_truth(freeze_path: Path, archive_path: Path, output_root: Path) -> dict[str, Any]:
    """Extract only the three frozen ground-truth members, without parsing them."""

    freeze = _verify_freeze(freeze_path)
    if sha256(archive_path) != ARCHIVE_SHA256:
        raise Phase29Error("archive SHA256 mismatch")
    metadata = _phase24_metadata()
    route_reports: dict[str, Any] = {}
    with zipfile.ZipFile(archive_path) as archive:
        for dataset_id in ROUTES:
            route, phone = dataset_id.split("/", 1)
            row = metadata.get(dataset_id)
            if not isinstance(row, dict) or not isinstance(row.get("ground_truth"), dict):
                raise Phase29Error(f"ground-truth central metadata missing: {dataset_id}")
            expected = row["ground_truth"]
            member_name = expected.get("name")
            if not isinstance(member_name, str) or Path(member_name).suffix.lower() != ".csv" or Path(member_name).name != "ground_truth.csv":
                raise Phase29Error(f"unexpected ground-truth member name: {dataset_id}")
            if ".mat" in member_name.lower():
                raise Phase29Error("MAT member rejected before archive open")
            try:
                info = archive.getinfo(member_name)
            except KeyError as exc:
                raise Phase29Error(f"missing archive member: {member_name}") from exc
            if info.is_dir() or info.file_size != expected.get("file_size") or f"{info.CRC:08x}" != expected.get("crc32_hex"):
                raise Phase29Error(f"ground-truth central metadata mismatch: {member_name}")
            destination = output_root / "truth" / route / phone / "ground_truth.csv"
            if destination.exists():
                raise Phase29Error(f"refusing to overwrite materialized truth: {destination}")
            destination.parent.mkdir(parents=True, exist_ok=True)
            temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
            if temporary.exists():
                raise Phase29Error(f"stale truth temporary file: {temporary}")
            digest = hashlib.sha256()
            size = 0
            try:
                with archive.open(info, "r") as source, temporary.open("xb") as target:
                    for chunk in iter(lambda: source.read(1024 * 1024), b""):
                        if not chunk:
                            break
                        target.write(chunk)
                        digest.update(chunk)
                        size += len(chunk)
                    target.flush()
                    os.fsync(target.fileno())
                if size != info.file_size:
                    raise Phase29Error(f"truth member size changed while extracting: {dataset_id}")
                os.replace(temporary, destination)
            finally:
                try:
                    temporary.unlink()
                except FileNotFoundError:
                    pass
            route_reports[dataset_id] = {
                "path": str(destination.relative_to(ROOT)),
                "member": member_name,
                "file_size": size,
                "compressed_size": info.compress_size,
                "crc32_hex": f"{info.CRC:08x}",
                "sha256": digest.hexdigest(),
                "parsed": False,
                "mat_opened": False,
            }
    report = {
        "schema_version": MATERIALIZATION_SCHEMA,
        "status": "truth-materialized-not-parsed",
        "freeze_record": {"path": str(freeze_path.relative_to(ROOT)), "sha256": sha256(freeze_path)},
        "archive": {"path": str(archive_path.relative_to(ROOT)), "sha256": ARCHIVE_SHA256},
        "routes": route_reports,
        "truth_materialized_count": len(route_reports),
        "truth_parse_count": 0,
        "mat_member_opened": False,
        "validation_truth_materialized": False,
        "future_holdout_truth_materialized": False,
    }
    atomic_json(output_root / "truth_materialization.json", report)
    return report


def _parse_float(raw: str | None, field: str, line: int) -> float:
    try:
        value = float((raw or "").strip())
    except ValueError as exc:
        raise Phase29Error(f"truth line {line}: invalid {field}") from exc
    if not math.isfinite(value):
        raise Phase29Error(f"truth line {line}: non-finite {field}")
    return value


def _read_truth_once(path: Path, default_phone: str) -> tuple[dict[tuple[str, int], tuple[float, float, float]], str, int]:
    """Read and hash one truth file through one file descriptor."""

    if path.suffix.lower() != ".csv":
        raise Phase29Error("truth input must be CSV")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise Phase29Error(f"failed to read truth: {path}") from exc
    digest = hashlib.sha256(payload).hexdigest()
    truth: dict[tuple[str, int], tuple[float, float, float]] = {}
    try:
        text = payload.decode("utf-8-sig")
        reader = csv.DictReader(io.StringIO(text, newline=""))
        fields = list(reader.fieldnames or ())
        required = {"UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"}
        if not required.issubset(fields):
            raise Phase29Error(f"truth missing fields: {sorted(required - set(fields))}")
        has_phone = "phone" in fields
        for line, raw in enumerate(reader, start=2):
            if None in raw:
                raise Phase29Error(f"truth line {line}: extra columns")
            phone = (raw.get("phone") or default_phone).strip()
            if phone != default_phone or not phone:
                raise Phase29Error(f"truth line {line}: phone mismatch")
            try:
                timestamp = int((raw.get("UnixTimeMillis") or "").strip())
            except ValueError as exc:
                raise Phase29Error(f"truth line {line}: invalid UnixTimeMillis") from exc
            if timestamp < 0:
                raise Phase29Error(f"truth line {line}: negative timestamp")
            latitude = _parse_float(raw.get("LatitudeDegrees"), "LatitudeDegrees", line)
            longitude = _parse_float(raw.get("LongitudeDegrees"), "LongitudeDegrees", line)
            altitude = _parse_float(raw.get("AltitudeMeters"), "AltitudeMeters", line)
            if not -90.0 <= latitude <= 90.0 or not -180.0 <= longitude <= 180.0:
                raise Phase29Error(f"truth line {line}: coordinate out of range")
            key = (phone, timestamp)
            if key in truth:
                raise Phase29Error(f"truth line {line}: duplicate key {key!r}")
            truth[key] = (latitude, longitude, altitude)
    except UnicodeDecodeError as exc:
        raise Phase29Error(f"truth is not UTF-8 CSV: {path}") from exc
    if not truth:
        raise Phase29Error(f"truth is empty: {path}")
    return truth, digest, len(payload)


def _prediction_rows(path: Path, expected_phone: str) -> list[kaggle.CoordinateRow]:
    if path.suffix.lower() == ".mat":
        raise Phase29Error("MAT prediction is forbidden")
    try:
        rows = kaggle._read_submission(path)
    except ValueError as exc:
        raise Phase29Error(str(exc)) from exc
    if any(row.phone != expected_phone for row in rows):
        raise Phase29Error(f"prediction phone mismatch: {path}")
    return rows


def _percentile(values: list[float], fraction: float, method: str) -> float:
    if method == "linear_n_minus_1":
        return kaggle._percentile_linear_n_minus_1(values, fraction)
    if method == "nearest_rank_ceiling":
        return kaggle._percentile_nearest_rank_ceiling(values, fraction)
    raise Phase29Error(f"unknown percentile method: {method}")


def _transition_speed(rows: list[kaggle.CoordinateRow]) -> dict[str, Any]:
    ordered = sorted(rows, key=lambda row: row.timestamp)
    speeds: list[float] = []
    max_transition: dict[str, Any] | None = None
    for previous, current in zip(ordered, ordered[1:]):
        dt = (current.timestamp - previous.timestamp) / 1000.0
        if dt <= 0.0:
            raise Phase29Error("prediction timestamps are not strictly increasing")
        displacement = kaggle._haversine_horizontal_distance_m(
            previous.latitude,
            previous.longitude,
            current.latitude,
            current.longitude,
        )
        speed = displacement / dt
        if not math.isfinite(speed):
            raise Phase29Error("prediction transition speed is non-finite")
        speeds.append(speed)
        if max_transition is None or speed > float(max_transition["speed_mps"]):
            max_transition = {
                "from_timestamp": previous.timestamp,
                "to_timestamp": current.timestamp,
                "speed_mps": speed,
                "gap_s": dt,
            }
    return {
        "transition_count": len(speeds),
        "max_speed_mps": max(speeds) if speeds else 0.0,
        "max_transition": max_transition,
        "over_70_mps_count": sum(speed > MAX_CONTINUITY_SPEED_MPS for speed in speeds),
    }


def _lane_metrics(
    rows: list[kaggle.CoordinateRow],
    truth: dict[tuple[str, int], tuple[float, float, float]],
    shared_keys: set[tuple[str, int]],
) -> dict[str, Any]:
    by_key = {(row.phone, row.timestamp): row for row in rows}
    truth_keys = set(truth)
    matched_truth_keys = set(by_key) & truth_keys
    if not shared_keys.issubset(matched_truth_keys):
        raise Phase29Error("shared scoring key is absent from lane truth intersection")
    if not shared_keys:
        raise Phase29Error("control/candidate common scoring key set is empty")
    wgs84: list[float] = []
    haversine: list[float] = []
    for key in sorted(shared_keys, key=lambda value: value[1]):
        predicted = by_key[key]
        latitude, longitude, altitude = truth[key]
        wgs84.append(kaggle._wgs84_horizontal_distance_m(predicted.latitude, predicted.longitude, latitude, longitude))
        haversine.append(kaggle._haversine_horizontal_distance_m(predicted.latitude, predicted.longitude, latitude, longitude))
    # The frozen native submission contract contains latitude/longitude only;
    # it has no predicted height.  Do not manufacture a vertical estimate from
    # truth altitude or an ellipsoid surface.  Keep the field explicit null so
    # the development gate fails closed instead of claiming a V comparison.
    def pair(values: list[float], method: str) -> dict[str, float]:
        return {"p50_m": _percentile(values, 0.50, method), "p95_m": _percentile(values, 0.95, method)}

    variants: dict[str, float] = {}
    for distance_name, values in (("wgs84_vincenty", wgs84), ("haversine_sphere", haversine)):
        for percentile_name in kaggle.PERCENTILE_VARIANT_IDS:
            p50 = _percentile(values, 0.50, percentile_name)
            p95 = _percentile(values, 0.95, percentile_name)
            variants[f"{distance_name}__{percentile_name}"] = (p50 + p95) / 2.0
    continuity = _transition_speed(rows)
    return {
        "prediction_rows": len(rows),
        "truth_rows": len(truth),
        "lane_truth_matched_rows": len(matched_truth_keys),
        "shared_scored_rows": len(shared_keys),
        "availability_ratio": len(matched_truth_keys) / len(truth),
        "truth_coverage_ratio": len(shared_keys) / len(truth),
        "horizontal_wgs84_m": pair(wgs84, "linear_n_minus_1"),
        "horizontal_haversine_m": pair(haversine, "linear_n_minus_1"),
        "vertical_p95_abs_m": None,
        "vertical_metric_status": "not-evaluable-submission-has-no-height-column",
        "kaggle_diagnostic_score_variants_m": variants,
        "kaggle_diagnostic_mean_m": sum(variants.values()) / len(variants),
        "continuity": continuity,
    }


def _lane_metrics_with_altitude(
    rows: list[kaggle.CoordinateRow],
    truth: dict[tuple[str, int], tuple[float, float, float]],
    shared_keys: set[tuple[str, int]],
) -> dict[str, Any]:
    """Score a common key set while preserving the no-height limitation."""

    return _lane_metrics(rows, truth, shared_keys)


def _compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["kaggle_diagnostic_score_variants_m"][key] >= control["kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            failures.append(f"{key}_not_strictly_improved")
    for path, label in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        current: Any = candidate
        reference: Any = control
        for key in path:
            current = current[key]
            reference = reference[key]
        if current is None or reference is None:
            if path == ("vertical_p95_abs_m",):
                failures.append("vertical_metric_unavailable")
            continue
        if current > reference + TOLERANCE:
            failures.append(label)
    for key in ("availability_ratio", "truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["continuity"]["over_70_mps_count"] > control["continuity"]["over_70_mps_count"]:
        failures.append("continuity_regression")
    return {"passed": not failures, "failures": failures}


def _aggregate(route_metrics: list[dict[str, Any]]) -> dict[str, Any]:
    if not route_metrics:
        raise Phase29Error("no route metrics")
    return {
        "route_count": len(route_metrics),
        "mean_availability_ratio": sum(row["availability_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_truth_coverage_ratio": sum(row["truth_coverage_ratio"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p50_m": sum(row["horizontal_wgs84_m"]["p50_m"] for row in route_metrics) / len(route_metrics),
        "mean_horizontal_wgs84_p95_m": sum(row["horizontal_wgs84_m"]["p95_m"] for row in route_metrics) / len(route_metrics),
        "mean_vertical_p95_abs_m": (
            sum(row["vertical_p95_abs_m"] for row in route_metrics) / len(route_metrics)
            if all(row["vertical_p95_abs_m"] is not None for row in route_metrics)
            else None
        ),
        "mean_kaggle_diagnostic_score_variants_m": {
            key: sum(row["kaggle_diagnostic_score_variants_m"][key] for row in route_metrics) / len(route_metrics)
            for key in DIAGNOSTIC_KEYS
        },
    }


def _aggregate_gate(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        if candidate["mean_kaggle_diagnostic_score_variants_m"][key] >= control["mean_kaggle_diagnostic_score_variants_m"][key] - TOLERANCE:
            failures.append(f"{key}_not_strictly_improved")
    for key in ("mean_availability_ratio", "mean_truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    for key, label in (
        ("mean_horizontal_wgs84_p50_m", "h_p50_regression"),
        ("mean_horizontal_wgs84_p95_m", "h_p95_regression"),
        ("mean_vertical_p95_abs_m", "v_p95_regression"),
    ):
        if candidate[key] is None or control[key] is None:
            if key == "mean_vertical_p95_abs_m":
                failures.append("vertical_metric_unavailable")
            continue
        if candidate[key] > control[key] + TOLERANCE:
            failures.append(label)
    return {"passed": not failures, "failures": failures}


def score(freeze_path: Path, output_root: Path) -> dict[str, Any]:
    started = time.perf_counter()
    freeze = _verify_freeze(freeze_path)
    materialization_path = output_root / "truth_materialization.json"
    materialization = load_json(materialization_path, "Phase29 truth materialization")
    if materialization.get("schema_version") != MATERIALIZATION_SCHEMA or materialization.get("truth_parse_count") != 0:
        raise Phase29Error("truth materialization is not sealed before parse")
    if materialization.get("validation_truth_materialized") is not False or materialization.get("future_holdout_truth_materialized") is not False:
        raise Phase29Error("validation/holdout truth was materialized")
    route_reports: dict[str, Any] = {}
    control_aggregate_rows: list[dict[str, Any]] = []
    candidate_aggregate_rows: list[dict[str, Any]] = []
    truth_read_count = 0
    for dataset_id in ROUTES:
        route, phone = dataset_id.split("/", 1)
        truth_info = materialization.get("routes", {}).get(dataset_id)
        if not isinstance(truth_info, dict):
            raise Phase29Error(f"missing materialized truth metadata: {dataset_id}")
        truth_path = ROOT / str(truth_info.get("path"))
        if truth_path.suffix.lower() != ".csv":
            raise Phase29Error("truth path is not CSV")
        truth, truth_hash, truth_bytes = _read_truth_once(truth_path, dataset_id)
        truth_read_count += 1
        if truth_hash != truth_info.get("sha256"):
            raise Phase29Error(f"truth hash changed since materialization: {dataset_id}")
        artifacts = freeze["sealed_route_artifacts"][dataset_id]
        control_rows = _prediction_rows(ROOT / artifacts["control"]["submission"], dataset_id)
        candidate_rows = _prediction_rows(ROOT / artifacts["candidate"]["submission"], dataset_id)
        control_keys = {(row.phone, row.timestamp) for row in control_rows}
        candidate_keys = {(row.phone, row.timestamp) for row in candidate_rows}
        truth_keys = set(truth)
        control_truth_keys = control_keys & truth_keys
        candidate_truth_keys = candidate_keys & truth_keys
        shared_keys = control_truth_keys & candidate_truth_keys
        if shared_keys != control_truth_keys or shared_keys != candidate_truth_keys:
            raise Phase29Error(f"control/candidate truth-matched key sets differ: {dataset_id}")
        control_metrics = _lane_metrics_with_altitude(control_rows, truth, shared_keys)
        candidate_metrics = _lane_metrics_with_altitude(candidate_rows, truth, shared_keys)
        gate = _compare(candidate_metrics, control_metrics)
        route_reports[dataset_id] = {
            "truth": {
                "path": truth_info["path"],
                "sha256": truth_hash,
                "bytes": truth_bytes,
                "read_count": 1,
            },
            "key_set": {
                "truth_keys": len(truth_keys),
                "control_prediction_keys": len(control_keys),
                "candidate_prediction_keys": len(candidate_keys),
                "control_truth_matched_keys": len(control_truth_keys),
                "candidate_truth_matched_keys": len(candidate_truth_keys),
                "shared_scored_keys": len(shared_keys),
                "same_matched_key_set": True,
            },
            "control": control_metrics,
            "candidate": candidate_metrics,
            "gate": gate,
            "artifact_hashes": {
                "control_submission_sha256": sha256(ROOT / artifacts["control"]["submission"]),
                "control_summary_sha256": sha256(ROOT / artifacts["control"]["summary"]),
                "candidate_submission_sha256": sha256(ROOT / artifacts["candidate"]["submission"]),
                "candidate_summary_sha256": sha256(ROOT / artifacts["candidate"]["summary"]),
            },
        }
        control_aggregate_rows.append(control_metrics)
        candidate_aggregate_rows.append(candidate_metrics)
    control_aggregate = _aggregate(control_aggregate_rows)
    candidate_aggregate = _aggregate(candidate_aggregate_rows)
    aggregate_gate = _aggregate_gate(candidate_aggregate, control_aggregate)
    route_gate = all(row["gate"]["passed"] for row in route_reports.values())
    train_passed = route_gate and aggregate_gate["passed"]
    report = {
        "schema_version": RESULT_SCHEMA,
        "status": "train-pass" if train_passed else "no-go-train-gate",
        "phase": 29,
        "candidate_id": freeze["inference_contract"]["candidate_id"],
        "control_id": freeze["inference_contract"]["control_id"],
        "truth_free_artifacts_verified_before_truth": True,
        "truth_open_count": truth_read_count,
        "truth_read_count_per_route": 1,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "routes": route_reports,
        "aggregate": {"control": control_aggregate, "candidate": candidate_aggregate, "gate": aggregate_gate},
        "train_gate": {"routewise_passed": route_gate, "aggregate_passed": aggregate_gate["passed"], "passed": train_passed},
        "validation_policy": {"opened": False, "fresh_validation": FRESH_VALIDATION, "future_holdout": FUTURE_HOLDOUT},
        "policy": {"post_score_tuning": False, "kaggle_or_token_access": False, "mat_read_or_generated": False, "test_truth_read": False},
        "runtime": {"wall_seconds": time.perf_counter() - started},
    }
    atomic_json(output_root / "train_evaluation.json", report)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "report": {"path": str((output_root / "train_evaluation.json").relative_to(ROOT)), "sha256": sha256(output_root / "train_evaluation.json")},
        "freeze_record": {"path": str(freeze_path.relative_to(ROOT)), "sha256": sha256(freeze_path)},
        "truth_materialization": {"path": str(materialization_path.relative_to(ROOT)), "sha256": sha256(materialization_path)},
        "truth_open_count": truth_read_count,
        "fresh_validation_truth_open_count": 0,
        "future_holdout_truth_open_count": 0,
        "post_score_tuning": False,
        "mat_read_or_generated": False,
    }
    atomic_json(output_root / "train_evaluation.manifest.json", manifest)
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("materialize-truth", "score"))
    parser.add_argument("--freeze", type=Path, default=FREEZE)
    parser.add_argument("--archive", type=Path, default=ARCHIVE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        if args.operation == "materialize-truth":
            report = materialize_truth(args.freeze, args.archive, args.output_root)
        else:
            report = score(args.freeze, args.output_root)
    except (OSError, Phase29Error, zipfile.BadZipFile) as exc:
        print(f"phase29: {exc}", file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
