#!/usr/bin/env python3
"""Score one sealed window-stitch artifact on an existing development train route.

The truth-free stitch is verified completely before the single historical
development truth file is read.  This evaluator is deliberately fixed to the
route used by the stitch experiment and refuses validation, holdout, test, or
leaderboard inputs.  It does not select parameters or rerun a solver.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from typing import Any


_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402

ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_native_fgo_eval as native_eval  # noqa: E402
import gnss_smartphone_native_fgo_pdc_bridge_train_score as prior_score  # noqa: E402
import gnss_smartphone_native_fgo_pdc_windowed_stitch as stitch  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402


SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-windowed-stitch-score.v1"
DATASET_ID = "2021-03-16-18-59-us-ca-mtv-a/pixel5"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_windowed_stitch_freeze_v1.json"
STITCH_ROOT = ROOT / "output/smartphone-r5/native-fgo-pdc-windowed-v2-stitch"
STITCH_MANIFEST = STITCH_ROOT / "stitch_manifest.json"
CANDIDATE_POS = STITCH_ROOT / "windowed_v2.pos"
BASELINE_POS = stitch.DEFAULT_FALLBACK
DEVICE = ROOT / "output/smartphone-r5/native-fgo-v2-processed/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5/inputs/device_gnss.csv"
TRUTH = ROOT / "output/smartphone-r5/native-fgo-carrier-compatibility-v1/train_truth/2021-03-16-18-59-us-ca-mtv-a__pixel5/ground_truth.csv"
TRUTH_SHA256 = "7c84ed6a80b1bbb08c0ffad57493513833b9d5474e22a43c5a44da82824ee22d"
EXPECTED_FREEZE_SHA256 = "40ef733d28cebd7d72710a122a2ec5cfe72a6c97a93480eb41a582b4f990dac6"
EXPECTED_STITCH_MANIFEST_SHA256 = "39e4c99681c604e37e41c0443f397a451875245156f42e10c95e60f369e6dbdb"
EXPECTED_STITCH_OUTPUT_SHA256 = "e9be773d6c7d10323f10c3ece4d4e1e5ed1edc7568c4b9a5221a6cb856a8105d"
LEAP_SECONDS = 18
MATCH_TOLERANCE_MS = 100
TOLERANCE = 1e-12
DIAGNOSTIC_KEYS = tuple(native_eval.DIAGNOSTIC_KEYS)
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-pdc-windowed-v2-stitch-score-v1"


class StitchScoreError(ValueError):
    """Raised when the sealed score contract is invalid."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise StitchScoreError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path.resolve())


def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def _metric(metrics: dict[str, Any], path: tuple[str, ...]) -> float:
    value: Any = metrics
    for key in path:
        value = value[key]
    if value is None or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        return math.inf
    return float(value)


def _verify_truth_free_artifacts() -> dict[str, Any]:
    freeze = stitch.verify_freeze(FREEZE)
    observed_freeze = _sha256(FREEZE)
    if observed_freeze != EXPECTED_FREEZE_SHA256:
        raise StitchScoreError("stitch freeze hash differs from the score authorization")
    if not STITCH_MANIFEST.is_file() or not CANDIDATE_POS.is_file():
        raise StitchScoreError("sealed stitch output is missing")
    observed_manifest = _sha256(STITCH_MANIFEST)
    observed_output = _sha256(CANDIDATE_POS)
    if observed_manifest != EXPECTED_STITCH_MANIFEST_SHA256:
        raise StitchScoreError("stitch manifest hash differs from the sealed score input")
    if observed_output != EXPECTED_STITCH_OUTPUT_SHA256:
        raise StitchScoreError("stitch position hash differs from the sealed score input")
    manifest = json.loads(STITCH_MANIFEST.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict) or manifest.get("schema_version") != stitch.SCHEMA:
        raise StitchScoreError("stitch manifest schema mismatch")
    if manifest.get("status") != "truth-free-windowed-pdc-stitch-sealed":
        raise StitchScoreError("stitch output is not sealed")
    policy = manifest.get("truth_policy")
    if not isinstance(policy, dict) or any(
        policy.get(key) is not expected
        for key, expected in (
            ("truth_free", True),
            ("truth_opened", False),
            ("validation_opened", False),
            ("holdout_opened", False),
            ("test_data_used", False),
            ("leaderboard_used", False),
        )
    ):
        raise StitchScoreError("stitch truth policy is not closed")
    if manifest.get("output_sha256") != observed_output:
        raise StitchScoreError("stitch output hash is not bound by its manifest")
    if not manifest.get("structural_gate", {}).get("passed"):
        raise StitchScoreError("development score is not permitted before structural pass")
    candidate = stitch._load_v1_manifest(stitch.DEFAULT_V1_ROOT)
    if _sha256(stitch.DEFAULT_V1_ROOT / "windowed_manifest.json") != freeze["window_contract"]["v1_manifest_sha256"]:
        raise StitchScoreError("v1 manifest changed before development score")
    # Strict parsers independently verify duplicate keys, finite ECEF, and
    # physical Earth bounds before truth is read.
    candidate_rows = stitch.windowed.read_pos(CANDIDATE_POS)
    baseline_rows = stitch.windowed.read_pos(BASELINE_POS)
    if [row.key for row in candidate_rows] != [row.key for row in baseline_rows]:
        raise StitchScoreError("candidate/baseline key order mismatch")
    if len(candidate_rows) != len(baseline_rows):
        raise StitchScoreError("candidate/baseline row count mismatch")
    if not DEVICE.is_file() or not TRUTH.is_file():
        raise StitchScoreError("fixed development inputs are missing")
    return {
        "freeze_sha256": observed_freeze,
        "stitch_manifest_sha256": observed_manifest,
        "candidate_output_sha256": observed_output,
        "candidate_rows": len(candidate_rows),
        "baseline_rows": len(baseline_rows),
        "v1_manifest_sha256": _sha256(stitch.DEFAULT_V1_ROOT / "windowed_manifest.json"),
        "freeze_status": freeze.get("status"),
        "truth_free_manifest": candidate.get("truth_policy"),
    }


def _score_position(path: Path, device_epochs: list[int], truth: dict[int, tuple[float, float, float]]) -> dict[str, Any]:
    positions = smoother._read_positions(path, LEAP_SECONDS)
    if any(row.timestamp_ms not in set(device_epochs) for row in positions):
        raise StitchScoreError(f"position timestamp is not in fixed device epoch set: {path}")
    return smoother_eval._score_rows(
        native_eval._raw_rows(positions),
        {row.timestamp_ms: row for row in positions},
        truth,
        0,
        len(device_epochs),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )


def _gate(candidate: dict[str, Any], baseline: dict[str, Any]) -> dict[str, Any]:
    failures: list[str] = []
    for path, label in (
        (("availability_ratio",), "availability_regression"),
        (("truth_coverage_ratio",), "truth_coverage_regression"),
        (("horizontal_wgs84_m", "p50_m"), "h_p50_regression"),
        (("horizontal_wgs84_m", "p95_m"), "h_p95_regression"),
        (("vertical_p95_abs_m",), "v_p95_regression"),
    ):
        if path[0] in ("availability_ratio", "truth_coverage_ratio"):
            if _metric(candidate, path) + TOLERANCE < _metric(baseline, path):
                failures.append(label)
        elif _metric(candidate, path) > _metric(baseline, path) + TOLERANCE:
            failures.append(label)
    for key in DIAGNOSTIC_KEYS:
        if _metric(candidate, ("kaggle_diagnostic_score_variants_m", key)) >= _metric(
            baseline, ("kaggle_diagnostic_score_variants_m", key)
        ) - TOLERANCE:
            failures.append(f"{key}_not_strictly_better")
    return {"passed": not failures, "failures": failures}


def score_once(output_root: Path = DEFAULT_OUTPUT) -> dict[str, Any]:
    if output_root.exists():
        raise StitchScoreError(f"score output already exists; refusing a second score: {output_root}")
    started = time.perf_counter()
    sealed = _verify_truth_free_artifacts()
    device_epochs = smoother._read_device_epochs(DEVICE, 0)
    # This is the only truth read in this command.  The helper hashes and
    # parses one byte buffer, preserving the prior development truth hash.
    truth, truth_hash = prior_score._read_truth_once(TRUTH, TRUTH_SHA256)
    candidate = _score_position(CANDIDATE_POS, device_epochs, truth)
    baseline = _score_position(BASELINE_POS, device_epochs, truth)
    gate = _gate(candidate, baseline)
    report: dict[str, Any] = {
        "schema_version": SCHEMA,
        "status": "train-pass" if gate["passed"] else "no-go-train-gate",
        "candidate": "native_fgo_pdc_triangular_ecef_overlap_stitch_v1",
        "baseline": "sealed_v5_native_fgo_route_position",
        "route": DATASET_ID,
        "truth_policy": {
            "truth_free_artifacts_verified_before_truth": True,
            "development_only": True,
            "truth_read_count": 1,
            "truth_sha256": truth_hash,
            "fresh_validation_truth_read_count": 0,
            "holdout_truth_read_count": 0,
            "test_truth_read_count": 0,
            "leaderboard_used_for_tuning": False,
            "token_access": False,
            "external_mutation": False,
            "no_post_score_parameter_tuning": True,
        },
        "sealed_inputs": sealed,
        "metrics": {"candidate": candidate, "baseline": baseline},
        "gate": gate,
        "runtime": {"wall_seconds": time.perf_counter() - started},
        "production_policy": {
            "production_default_changed": False,
            "v5_artifact_changed": False,
            "validation_or_holdout_opened": False,
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    report_path = output_root / "stitch_train_evaluation.json"
    _atomic_json(report_path, report)
    manifest = {
        "schema_version": SCHEMA + "-manifest",
        "status": report["status"],
        "report": {"path": _relative(report_path), "sha256": _sha256(report_path), "bytes": report_path.stat().st_size},
        "candidate_output_sha256": sealed["candidate_output_sha256"],
        "stitch_manifest_sha256": sealed["stitch_manifest_sha256"],
        "freeze_sha256": sealed["freeze_sha256"],
        "truth_read_count": 1,
        "truth_sha256": truth_hash,
        "fresh_validation_truth_read_count": 0,
        "holdout_truth_read_count": 0,
        "no_post_score_parameter_tuning": True,
    }
    manifest_path = output_root / "stitch_train_evaluation.manifest.json"
    _atomic_json(manifest_path, manifest)
    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(argv)
    try:
        report = score_once(args.output_root if args.output_root.is_absolute() else ROOT / args.output_root)
    except (OSError, ValueError, StitchScoreError) as exc:
        print(f"windowed stitch score failed closed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": report["status"], "gate": report["gate"], "truth_read_count": report["truth_policy"]["truth_read_count"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
