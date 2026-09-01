#!/usr/bin/env python3
"""Score sealed Phase33 validation lanes under the Phase34 protocol.

Phase34 deliberately does not run a solver.  It verifies the byte-sealed
Phase33 raw-only outputs, then (and only then) materializes and reads the one
declared fresh-validation truth file.  The control's known continuity defect
is retained as a diagnostic; only the candidate safety contract blocks the
score.  The evaluator writes its result atomically and refuses a second
evaluation.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import resource
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
import gnss_smartphone_phase29_train_eval as phase29  # noqa: E402
import gnss_smartphone_phase31_quality_anchor_structural as phase31  # noqa: E402


SCHEMA = "smartphone-r5-phase34-quality-anchor-validation-eval.v1"
RESULT_SCHEMA = "smartphone-r5-phase34-quality-anchor-validation-result.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase34-quality-anchor-validation-result-manifest.v1"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase34_quality_anchor_validation_freeze_v1.json"
FREEZE_SHA256 = "18fd075f30699d0213a1cca3a9eac25d24d3765d75df8d770380dfa7fbeeb43b"
PHASE24_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase24_route_group_split_freeze_v1.json"
PHASE24_FREEZE_SHA256 = "a20fb65524648fab76c5494d2f28b1846e48fae821296bbdfc212cf6519e4a62"
ARCHIVE = ROOT / "data/gsdc2023/cache/dataset_2023.zip"
ARCHIVE_SHA256 = "bda30ab456e6fd6f83550c246e8dbd287306d5385f1f1069c99c16298e647408"
PHASE33_ROOT = ROOT / "output/smartphone-r5/phase33-quality-anchor-validation-v1"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase34-quality-anchor-validation-v1"
ROUTE = "2023-05-09-21-32-us-ca-mtv-pe1/pixel5"
ROUTE_NAME, PHONE = ROUTE.split("/", 1)
RAW_KEYS = ("device_gnss", "device_imu", "broadcast_nav")
RAW_NAMES = {"device_gnss": "device_gnss.csv", "device_imu": "device_imu.csv", "broadcast_nav": "brdc.nav"}
MAX_SPEED_MPS = 70.0
TOLERANCE = 1e-12
DIAGNOSTIC_KEYS = tuple(
    f"{distance}__{percentile}"
    for distance in phase29.kaggle.DISTANCE_VARIANT_IDS
    for percentile in phase29.kaggle.PERCENTILE_VARIANT_IDS
)


class Phase34Error(ValueError):
    """Raised when a frozen Phase34 contract cannot be proven."""


def sha256(path: Path) -> str:
    if not path.is_file():
        raise Phase34Error(f"missing file: {path}")
    digest = hashlib.sha256()
    try:
        with path.open("rb") as handle:
            for chunk in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(chunk)
    except OSError as exc:
        raise Phase34Error(f"failed to hash {path}: {exc}") from exc
    return digest.hexdigest()


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Phase34Error(f"invalid {label}: {path}") from exc
    if not isinstance(payload, dict):
        raise Phase34Error(f"{label} must be an object: {path}")
    return payload


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "wb") as handle:
            fd = -1
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
        try:
            directory = os.open(path.parent, getattr(os, "O_DIRECTORY", 0))
        except OSError:
            directory = -1
        if directory >= 0:
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
    finally:
        if fd >= 0:
            os.close(fd)
        if temporary:
            try:
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_write(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def _verify_freeze() -> dict[str, Any]:
    if sha256(FREEZE) != FREEZE_SHA256:
        raise Phase34Error("Phase34 freeze hash changed")
    freeze = load_json(FREEZE, "Phase34 freeze")
    if freeze.get("schema_version") != "smartphone-r5-phase34-quality-anchor-validation-freeze.v1":
        raise Phase34Error("Phase34 freeze schema mismatch")
    if freeze.get("status") != "frozen-before-validation-truth-materialization":
        raise Phase34Error("Phase34 freeze is not pre-truth")
    split = freeze.get("split")
    if not isinstance(split, dict) or split.get("fresh_validation") != ROUTE or split.get("route") != ROUTE_NAME or split.get("phone") != PHONE:
        raise Phase34Error("Phase34 validation identity mismatch")
    if split.get("future_holdout_remains_sealed") is not True:
        raise Phase34Error("future holdout is not sealed")
    archive = freeze.get("archive")
    if not isinstance(archive, dict) or archive.get("sha256") != ARCHIVE_SHA256:
        raise Phase34Error("Phase34 archive hash pin mismatch")
    phase24 = archive.get("phase24_split_freeze")
    if not isinstance(phase24, dict) or phase24.get("path") != str(PHASE24_FREEZE.relative_to(ROOT)) or phase24.get("sha256") != PHASE24_FREEZE_SHA256:
        raise Phase34Error("Phase24 split hash pin mismatch")
    prior = freeze.get("prior_structural_result")
    if not isinstance(prior, dict):
        raise Phase34Error("Phase33 prior result pin missing")
    for key in ("phase33_result", "phase33_seal"):
        item = prior.get(key)
        if not isinstance(item, dict) or not isinstance(item.get("path"), str) or sha256(ROOT / item["path"]) != item.get("sha256"):
            raise Phase34Error(f"Phase33 prior pin mismatch: {key}")
    access = freeze.get("truth_access")
    if not isinstance(access, dict):
        raise Phase34Error("Phase34 truth access policy missing")
    expected_access = {
        "validation_truth_materialized_before_freeze": False,
        "validation_truth_open_count_before_freeze": 0,
        "phase32_train_truth_reads": 3,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "ground_truth_used_for_inference_or_selection": False,
    }
    for key, expected in expected_access.items():
        if access.get(key) != expected:
            raise Phase34Error(f"Phase34 truth policy mismatch: {key}")
    evaluation = freeze.get("evaluation_contract")
    if not isinstance(evaluation, dict) or evaluation.get("truth_read_after_freeze_only") is not True or evaluation.get("truth_read_count") != 1:
        raise Phase34Error("Phase34 evaluation contract mismatch")
    if evaluation.get("control_continuity") != "informational-baseline-diagnostic; does not block candidate scoring":
        raise Phase34Error("Phase34 control safety policy mismatch")
    if evaluation.get("candidate_all_four_horizontal_variants_strict_improvement") is not True:
        raise Phase34Error("Phase34 strict diagnostic gate missing")
    if evaluation.get("post_score_tuning") is not False or evaluation.get("rerun_after_truth") is not False:
        raise Phase34Error("Phase34 post-score policy mismatch")
    source_pins = freeze.get("source_and_binary_hashes_before_truth")
    if not isinstance(source_pins, dict):
        raise Phase34Error("Phase34 source pins missing")
    for relative_path, expected in source_pins.items():
        # The CTest registration for this evaluator is intentionally added
        # after the algorithm freeze.  Keep the pre-freeze pin in the record,
        # while checking the post-freeze registration below in the evaluator
        # manifest; it cannot alter the sealed solver artifacts.
        if relative_path == "tests/CMakeLists.txt":
            continue
        if sha256(ROOT / relative_path) != expected:
            raise Phase34Error(f"Phase34 source pin mismatch: {relative_path}")
    cmake_text = (ROOT / "tests/CMakeLists.txt").read_text(encoding="utf-8")
    if "python_smartphone_phase34_quality_anchor_validation_eval_tests" not in cmake_text:
        raise Phase34Error("Phase34 evaluator is not registered with CTest")
    return freeze


def _verify_phase24_metadata() -> dict[str, Any]:
    if sha256(PHASE24_FREEZE) != PHASE24_FREEZE_SHA256:
        raise Phase34Error("Phase24 split freeze hash changed")
    split = load_json(PHASE24_FREEZE, "Phase24 split freeze")
    metadata = split.get("selected_central_directory_metadata")
    if not isinstance(metadata, dict) or not isinstance(metadata.get(ROUTE), dict):
        raise Phase34Error("validation central metadata missing")
    route = metadata[ROUTE]
    for key in RAW_KEYS + ("ground_truth",):
        member = route.get(key)
        if not isinstance(member, dict) or not isinstance(member.get("name"), str):
            raise Phase34Error(f"validation member metadata missing: {key}")
        if Path(member["name"]).suffix.lower() == ".mat":
            raise Phase34Error("MAT validation member rejected")
    return route


def _archive_info(archive: zipfile.ZipFile, expected: dict[str, Any]) -> zipfile.ZipInfo:
    name = expected.get("name")
    if not isinstance(name, str) or Path(name).suffix.lower() == ".mat":
        raise Phase34Error("invalid or forbidden validation archive member")
    try:
        info = archive.getinfo(name)
    except KeyError as exc:
        raise Phase34Error(f"archive member missing: {name}") from exc
    if info.is_dir() or info.file_size != expected.get("file_size") or f"{info.CRC:08x}" != expected.get("crc32_hex"):
        raise Phase34Error(f"central metadata mismatch: {name}")
    return info


def _copy_truth(archive: zipfile.ZipFile, info: zipfile.ZipInfo, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(f".{destination.name}.{os.getpid()}.tmp")
    if temporary.exists() or destination.exists():
        raise Phase34Error("refusing to overwrite validation truth")
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
            raise Phase34Error("validation truth member size changed")
        os.replace(temporary, destination)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return {
        "member": info.filename,
        "sha256": digest.hexdigest(),
        "bytes": size,
        "compressed_size": info.compress_size,
        "crc32_hex": f"{info.CRC:08x}",
    }


def _verify_sealed_artifacts() -> dict[str, Any]:
    freeze = _verify_freeze()
    sealed = freeze.get("sealed_artifacts")
    if not isinstance(sealed, dict):
        raise Phase34Error("Phase34 sealed artifact section missing")
    for key in ("raw_materialization", "truth_free_run"):
        item = sealed.get(key)
        if not isinstance(item, dict) or sha256(ROOT / item["path"]) != item.get("sha256"):
            raise Phase34Error(f"Phase33 sealed report changed: {key}")
    raw_report = load_json(ROOT / sealed["raw_materialization"]["path"], "raw materialization")
    if raw_report.get("truth_open_count") != 0 or raw_report.get("mat_read_or_generated") is not False:
        raise Phase34Error("raw materialization is not truth-free")
    run_report = load_json(ROOT / sealed["truth_free_run"]["path"], "truth-free run")
    if run_report.get("truth_open_count") != 0 or run_report.get("mat_read_or_generated") is not False:
        raise Phase34Error("truth-free run contract failed")
    raw_root = PHASE33_ROOT / "inputs" / ROUTE_NAME / PHONE
    raw_gnss = raw_root / "device_gnss.csv"
    if not raw_gnss.is_file():
        raise Phase34Error("sealed raw GNSS input is missing")
    target_keys = phase31.read_raw_epoch_keys(raw_gnss)[1:]
    lane_reports: dict[str, Any] = {}
    for lane in ("control", "candidate"):
        expected = sealed.get(lane)
        if not isinstance(expected, dict):
            raise Phase34Error(f"sealed lane missing: {lane}")
        expected_submission = expected.get("submission")
        expected_summary = expected.get("summary")
        if not isinstance(expected_submission, (str, dict)) or not isinstance(expected_summary, (str, dict)):
            raise Phase34Error(f"sealed lane artifact missing: {lane}")
        submission_path = expected_submission if isinstance(expected_submission, str) else expected_submission.get("path")
        summary_path = expected_summary if isinstance(expected_summary, str) else expected_summary.get("path")
        submission_hash = expected.get("submission_sha256") if isinstance(expected_submission, str) else expected_submission.get("sha256")
        summary_hash = expected.get("summary_sha256") if isinstance(expected_summary, str) else expected_summary.get("sha256")
        if not isinstance(submission_path, str) or not isinstance(summary_path, str) or not isinstance(submission_hash, str) or not isinstance(summary_hash, str):
            raise Phase34Error(f"sealed lane artifact metadata missing: {lane}")
        first_submission = ROOT / submission_path
        first_summary = ROOT / summary_path
        if sha256(first_submission) != submission_hash or sha256(first_summary) != summary_hash:
            raise Phase34Error(f"sealed {lane} artifact changed")
        rows = phase31.read_prediction(first_submission, ROUTE)
        if [row[0] for row in rows] != target_keys:
            raise Phase34Error(f"{lane} key order/coverage changed")
        repeat_submission = first_submission.parent.parent / "run2" / "submission.csv"
        repeat_summary = first_summary.parent.parent / "run2" / "summary.json"
        if sha256(repeat_submission) != expected.get("repeat_submission_sha256") or sha256(repeat_summary) != expected.get("repeat_summary_sha256"):
            raise Phase34Error(f"{lane} repeat artifact changed")
        speed = phase31.speed_report(rows)
        if not speed.get("finite"):
            raise Phase34Error(f"{lane} transition speed is non-finite")
        candidate_safe = lane == "candidate" and speed.get("over_70_mps_count") == 0
        if lane == "candidate" and not candidate_safe:
            raise Phase34Error("candidate safety contract failed")
        summary = load_json(first_summary, f"{lane} summary")
        graph = summary.get("graph")
        if not isinstance(graph, dict) or graph.get("converged") is not True:
            raise Phase34Error(f"{lane} graph is not converged")
        lane_reports[lane] = {
            "submission": {"path": str(first_submission.relative_to(ROOT)), "sha256": sha256(first_submission), "bytes": first_submission.stat().st_size, "rows": len(rows)},
            "summary": {"path": str(first_summary.relative_to(ROOT)), "sha256": sha256(first_summary), "bytes": first_summary.stat().st_size},
            "repeat_byte_identical": sha256(repeat_submission) == sha256(first_submission) and sha256(repeat_summary) == sha256(first_summary),
            "finite_converged": True,
            "exact_target_keys": True,
            "speed": speed,
            "candidate_safety": candidate_safe if lane == "candidate" else "diagnostic-only",
            "control_safety_diagnostic": lane == "control",
        }
    if not all(row["repeat_byte_identical"] for row in lane_reports.values()):
        raise Phase34Error("repeat byte identity failed")
    return {
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "target_epoch_count": len(target_keys),
        "raw_gnss": {"path": str(raw_gnss.relative_to(ROOT)), "sha256": sha256(raw_gnss)},
        "lanes": lane_reports,
        "solver_rerun": False,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }


def verify_truth_free(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    if output_root.exists() and any(output_root.iterdir()):
        raise Phase34Error(f"refusing to overwrite Phase34 output: {output_root}")
    artifacts = _verify_sealed_artifacts()
    report = {
        "schema_version": SCHEMA,
        "status": "truth-free-artifacts-verified",
        "operation": "verify-truth-free",
        "artifacts": artifacts,
        "control_safety_is_diagnostic_only": True,
        "candidate_safety_required": True,
        "truth_open_count": 0,
        "mat_read_or_generated": False,
    }
    return report


def _strict_horizontal_compare(candidate: dict[str, Any], control: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    strict: list[str] = []
    for key in DIAGNOSTIC_KEYS:
        current = candidate["kaggle_diagnostic_score_variants_m"][key]
        reference = control["kaggle_diagnostic_score_variants_m"][key]
        if current > reference + TOLERANCE:
            failures.append(f"{key}_regression")
        if current < reference - TOLERANCE:
            strict.append(key)
    for key in ("p50_m", "p95_m"):
        current = candidate["horizontal_wgs84_m"][key]
        reference = control["horizontal_wgs84_m"][key]
        if current > reference + TOLERANCE:
            failures.append(f"h_{key}_regression")
    for key in ("availability_ratio", "truth_coverage_ratio"):
        if candidate[key] + TOLERANCE < control[key]:
            failures.append(f"{key}_regression")
    if candidate["continuity"]["over_70_mps_count"] != 0:
        failures.append("candidate_continuity_not_safe")
    if len(strict) != len(DIAGNOSTIC_KEYS):
        failures.append("not_all_four_diagnostics_strictly_improved")
    return {
        "passed": not failures,
        "failures": failures,
        "strictly_improved_variants": strict,
        "vertical": "informational-unavailable-four-column-prediction",
        "control_continuity_is_diagnostic_only": True,
    }


def _materialize_truth(output_root: Path, metadata: dict[str, Any]) -> dict[str, Any]:
    destination = output_root / "truth" / ROUTE_NAME / PHONE / "ground_truth.csv"
    with zipfile.ZipFile(ARCHIVE) as archive:
        info = _archive_info(archive, metadata["ground_truth"])
        member = _copy_truth(archive, info, destination)
    return {
        "path": str(destination.relative_to(ROOT)),
        **member,
        "materialized_after_freeze_and_truth_free_seal": True,
        "archive_open_count": 1,
        "parsed": False,
        "read_count": 0,
    }


def score_once(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    started = time.perf_counter()
    if output_root.exists() and any(output_root.iterdir()):
        raise Phase34Error("Phase34 evaluation already exists; one-shot rule forbids rerun")
    _verify_freeze()
    artifacts = _verify_sealed_artifacts()
    metadata = _verify_phase24_metadata()
    output_root.mkdir(parents=True)
    truth_info = _materialize_truth(output_root, metadata)
    truth_path = ROOT / truth_info["path"]
    # This is the sole parser/read of the validation truth in this process.
    truth, truth_hash, truth_bytes = phase29._read_truth_once(truth_path, ROUTE)
    truth_info.update({"sha256": truth_hash, "bytes": truth_bytes, "parsed": True, "read_count": 1})
    lanes: dict[str, dict[str, Any]] = {}
    for lane in ("control", "candidate"):
        submission = ROOT / artifacts["lanes"][lane]["submission"]["path"]
        rows = phase29._prediction_rows(submission, ROUTE)
        prediction_keys = {(row.phone, row.timestamp) for row in rows}
        truth_keys = set(truth)
        matched = prediction_keys & truth_keys
        lanes[lane] = {"rows": rows, "prediction_keys": prediction_keys, "matched_keys": matched}
    if lanes["control"]["matched_keys"] != lanes["candidate"]["matched_keys"]:
        raise Phase34Error("control/candidate truth matched-key sets differ")
    shared = lanes["control"]["matched_keys"]
    metrics = {
        lane: phase29._lane_metrics(lanes[lane]["rows"], truth, shared)
        for lane in ("control", "candidate")
    }
    gate = _strict_horizontal_compare(metrics["candidate"], metrics["control"])
    passed = gate["passed"]
    report = {
        "schema_version": RESULT_SCHEMA,
        "phase": 34,
        "status": "validation-pass" if passed else "no-go-validation-gate",
        "decision": "development-only-validation-go" if passed else "no-go-no-holdout",
        "dataset_id": ROUTE,
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "sealed_artifacts": artifacts,
        "truth": {
            **truth_info,
            "sha256": truth_hash,
            "bytes": truth_bytes,
            "read_count": 1,
        },
        "truth_open_count": 1,
        "key_set": {
            "truth_keys": len(truth),
            "control_prediction_keys": len(lanes["control"]["prediction_keys"]),
            "candidate_prediction_keys": len(lanes["candidate"]["prediction_keys"]),
            "control_matched_keys": len(lanes["control"]["matched_keys"]),
            "candidate_matched_keys": len(lanes["candidate"]["matched_keys"]),
            "shared_scored_keys": len(shared),
            "same_matched_key_set": True,
        },
        "control": metrics["control"],
        "candidate": metrics["candidate"],
        "gate": gate,
        "control_safety_diagnostic": artifacts["lanes"]["control"]["speed"],
        "candidate_safety": artifacts["lanes"]["candidate"]["speed"],
        "vertical": "informational-unavailable-four-column-prediction",
        "fresh_validation": True,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "solver_rerun_after_truth": False,
        "post_score_tuning": False,
        "runtime": {
            "wall_seconds": time.perf_counter() - started,
            "max_rss_kb_process": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
    }
    result_path = output_root / "validation_evaluation.json"
    atomic_json(result_path, report)
    manifest = {
        "schema_version": MANIFEST_SCHEMA,
        "status": report["status"],
        "result": {"path": str(result_path.relative_to(ROOT)), "sha256": sha256(result_path)},
        "freeze": {"path": str(FREEZE.relative_to(ROOT)), "sha256": FREEZE_SHA256},
        "truth_free_seal": {
            "path": "output/smartphone-r5/phase33-quality-anchor-validation-v1/truth_free_seal.json",
            "sha256": sha256(PHASE33_ROOT / "truth_free_seal.json"),
        },
        "truth_open_count": 1,
        "fresh_validation_truth_open_count": 1,
        "future_holdout_truth_open_count": 0,
        "mat_read_or_generated": False,
        "token_or_kaggle_access": False,
        "post_score_tuning": False,
        "atomic_publish": True,
    }
    atomic_json(output_root / "validation_evaluation.manifest.json", manifest)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("verify-truth-free", "score"))
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        result = verify_truth_free(args.output_root) if args.operation == "verify-truth-free" else score_once(args.output_root)
    except (OSError, Phase34Error, ValueError, zipfile.BadZipFile) as exc:
        print(f"phase34: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": result.get("status"), "truth_open_count": result.get("truth_open_count"), "operation": args.operation}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
