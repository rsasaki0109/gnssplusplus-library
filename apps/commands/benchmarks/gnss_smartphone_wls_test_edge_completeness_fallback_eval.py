#!/usr/bin/env python3
"""Truth-free leading/trailing edge constant-hold evaluation.

All masked outputs are generated and hashed before development truth is read.
The candidate is fixed: a leading/trailing unresolved key may hold the nearest
finite selected-source ECEF for at most ten seconds when the raw epoch clock
and derived segment identity remain continuous.  No velocity extrapolation or
parameter search is performed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import resource
import sys
import time
from typing import Any

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))

import gnss_smartphone_reacquisition_eval as reacquisition  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_test_batch as batch  # noqa: E402
import gnss_smartphone_wls_test_completeness_fallback_eval as v1_eval  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-test-edge-completeness-fallback-evaluation.v2"
MASK_DURATIONS_S = (1, 2, 5, 10)
H_P50_MAX_REGRESSION_M = 0.01
H_P95_MAX_REGRESSION_M = 5.0
V_P95_MAX_REGRESSION_M = 5.0
DIAGNOSTIC_MAX_REGRESSION_M = 5.0
MATCH_TOLERANCE_MS = 100


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _artifact(path: Path) -> dict[str, Any]:
    return {"path": str(path), "sha256": _sha256(path), "bytes": path.stat().st_size}


def _absolute(raw: str) -> Path:
    path = Path(raw)
    return path if path.is_absolute() else ROOT / path


def _score(
    positions: list[smoother.PositionRow],
    device: Path,
    truth_values: dict[int, Any],
) -> dict[str, Any]:
    epochs = smoother._read_device_epochs(device, 1)
    if [row.timestamp_ms for row in positions] != epochs:
        raise ValueError("edge mask output keys differ from device epochs")
    rows = reacquisition._raw_rows(positions)
    return smoother_eval._score_rows(
        rows,
        {row.timestamp_ms: row for row in positions},
        truth_values,
        0,
        len(epochs),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )


def _edge_mask(
    positions: list[smoother.PositionRow],
    device: Path,
    duration_s: int,
    direction: str,
) -> tuple[list[smoother.PositionRow], dict[str, Any]]:
    if direction not in {"leading", "trailing"}:
        raise ValueError(f"invalid edge direction: {direction}")
    metadata = batch._read_epoch_clock_segments(device, skip_epochs=1)
    selected_keys = list(metadata["timestamps_ms"])
    if [row.timestamp_ms for row in positions] != selected_keys:
        raise ValueError("development WLS keys differ from device epoch keys")
    count = duration_s
    if len(positions) <= count:
        raise ValueError("edge mask removes every selected source")
    if direction == "leading":
        remaining = positions[count:]
        omitted = positions[:count]
        source = remaining[0]
    else:
        remaining = positions[:-count]
        omitted = positions[-count:]
        source = remaining[-1]
    state = {
        "a": {
            "positions": remaining,
            "position_timestamps": [row.timestamp_ms for row in remaining],
            "position_by_timestamp": {row.timestamp_ms: row for row in remaining},
            "selected_keys": selected_keys,
            "epoch_clock_segments": metadata,
            "native_fallback": None,
        }
    }
    fused, stats = batch._test_fuse_positions(
        state,
        allow_completeness_fallback=True,
        allow_edge_completeness_fallback=True,
    )
    output = fused["a"]
    if [row.timestamp_ms for row in output] != selected_keys:
        raise ValueError("edge hold did not preserve target keys")
    for row in output[:count] if direction == "leading" else output[-count:]:
        if row.ecef.tolist() != source.ecef.tolist():
            raise ValueError("edge hold changed the selected-source ECEF")
    edge_records = [
        entry
        for entry in stats["epoch_manifest"]
        if entry.get("source") == "wls_ecef_constant_edge_hold"
    ]
    expected_gap_ms = abs(source.timestamp_ms - omitted[0 if direction == "leading" else -1].timestamp_ms)
    if len(edge_records) != count:
        raise ValueError("edge hold did not account for every masked key")
    if max(int(entry["edge_hold_gap_ms"]) for entry in edge_records) != expected_gap_ms:
        raise ValueError("edge hold gap accounting differs")
    return output, {
        "source": "wls_ecef_constant_edge_hold",
        "direction": direction,
        "requested_mask_duration_s": duration_s,
        "omitted_key_count": count,
        "nearest_source_timestamp_ms": source.timestamp_ms,
        "max_edge_gap_ms": max(int(entry["edge_hold_gap_ms"]) for entry in edge_records),
        "edge_hold_epoch_count": len(edge_records),
        "clock_segment_continuity_verified": all(
            entry.get("edge_hold_continuity_verified") is True for entry in edge_records
        ),
        "velocity_extrapolation": False,
        "stats": {
            "edge_constant_hold_epoch_count": stats["edge_constant_hold_epoch_count"],
            "edge_constant_hold_gap_ms": stats["edge_constant_hold_gap_ms"],
            "edge_constant_hold_direction_counts": stats[
                "edge_constant_hold_direction_counts"
            ],
        },
    }


def _regression_failures(candidate: dict[str, Any], baseline: dict[str, Any]) -> list[str]:
    failures: list[str] = []
    for path, label, limit in (
        (("horizontal_wgs84_m", "p50_m"), "h_p50", H_P50_MAX_REGRESSION_M),
        (("horizontal_wgs84_m", "p95_m"), "h_p95", H_P95_MAX_REGRESSION_M),
        (("vertical_p95_abs_m",), "v_p95", V_P95_MAX_REGRESSION_M),
    ):
        value: Any = candidate
        reference: Any = baseline
        for key in path:
            value = value[key]
            reference = reference[key]
        if value is None or reference is None or float(value) > float(reference) + limit:
            failures.append(f"{label}_upper_bound")
    for key in candidate["kaggle_diagnostic_score_variants_m"]:
        value = float(candidate["kaggle_diagnostic_score_variants_m"][key])
        reference = float(baseline["kaggle_diagnostic_score_variants_m"][key])
        if value > reference + DIAGNOSTIC_MAX_REGRESSION_M:
            failures.append(f"{key}_upper_bound")
    if float(candidate["availability_ratio"]) + 1e-12 < float(baseline["availability_ratio"]):
        failures.append("availability_regression")
    if float(candidate["truth_coverage_ratio"]) + 1e-12 < float(baseline["truth_coverage_ratio"]):
        failures.append("truth_coverage_regression")
    return failures


def run(output_dir: Path) -> dict[str, Any]:
    if output_dir.exists() and any(output_dir.iterdir()):
        raise ValueError(f"refusing to overwrite edge evaluation output: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    route_reports: list[dict[str, Any]] = []

    # Phase 1: materialize only truth-free development artifacts and seal all
    # hashes before opening any development ground-truth file.
    for route in v1_eval.ROUTES:
        dataset_id = str(route["dataset_id"])
        wls_path = _absolute(str(route["wls"]))
        device_path = _absolute(str(route["device"]))
        positions = smoother._read_positions(wls_path, 18)
        route_dir = output_dir / "routes" / batch._safe_id(dataset_id)
        route_dir.mkdir(parents=True, exist_ok=True)
        scenarios: dict[str, tuple[list[smoother.PositionRow], dict[str, Any]]] = {
            "unmasked_baseline": (positions, {"source": "wls_raw"})
        }
        for duration_s in MASK_DURATIONS_S:
            for direction in ("leading", "trailing"):
                name = f"{direction}_mask_{duration_s}s"
                scenarios[name] = _edge_mask(positions, device_path, duration_s, direction)
        scenario_artifacts: dict[str, Any] = {}
        for scenario, (rows, meta) in scenarios.items():
            position_path = route_dir / f"{scenario}.pos"
            wls._write_pos(position_path, rows)
            scenario_artifacts[scenario] = {
                "position": _artifact(position_path),
                "truth_free": True,
                "truth_used": False,
                "meta": meta,
            }
        route_reports.append(
            {
                "dataset_id": dataset_id,
                "wls": _artifact(wls_path),
                "device": _artifact(device_path),
                "truth_not_opened_in_phase": True,
                "scenarios": scenario_artifacts,
            }
        )
    truth_free_manifest = {
        "schema_version": "smartphone-r5-wls-test-edge-completeness-fallback-truth-free-manifest.v2",
        "status": "truth-free-edge-artifacts-sealed-before-development-truth-read",
        "truth_free": True,
        "truth_used": False,
        "candidate_contract": dict(batch.EDGE_COMPLETENESS_FALLBACK_CONTRACT),
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "routes": route_reports,
        "truth_policy": {"truth_open_count": 0, "ground_truth_members_read": False},
    }
    truth_free_manifest_path = output_dir / "truth_free_artifacts_manifest.json"
    batch._atomic_json(truth_free_manifest_path, truth_free_manifest)

    # Phase 2: read each already-materialized development truth once, after
    # all candidate outputs and their hashes are sealed.
    gate_failures: list[str] = []
    for route_report, route in zip(route_reports, v1_eval.ROUTES):
        dataset_id = str(route_report["dataset_id"])
        device_path = _absolute(str(route["device"]))
        truth_path = _absolute(str(route["truth"]))
        truth_values = smoother_eval._read_truth(truth_path)
        positions_by_scenario = {
            scenario: smoother._read_positions(Path(str(artifact["position"]["path"])), 18)
            for scenario, artifact in route_report["scenarios"].items()
        }
        baseline = _score(positions_by_scenario["unmasked_baseline"], device_path, truth_values)
        route_report["baseline_metrics"] = baseline
        route_report["scenario_metrics"] = {}
        for scenario, rows in positions_by_scenario.items():
            if scenario == "unmasked_baseline":
                continue
            metrics = _score(rows, device_path, truth_values)
            failures = _regression_failures(metrics, baseline)
            route_report["scenario_metrics"][scenario] = {
                "metrics": metrics,
                "gate_failures": failures,
                "passed": not failures,
            }
            gate_failures.extend(f"{dataset_id}:{failure}" for failure in failures)
    summary = {
        "route_count": len(route_reports),
        "scenario_count": sum(len(report["scenarios"]) for report in route_reports),
        "truth_open_count": len(route_reports),
        "truth_read_after_truth_free_hash_seal": True,
        "gate_failures": gate_failures,
        "passed": not gate_failures,
    }
    evaluation = {
        "schema_version": SCHEMA_VERSION,
        "status": "completed-one-shot-development-edge-mask-evaluation",
        "candidate_contract": {
            "candidate_search": False,
            "parameter_research": False,
            "fallback_policy": dict(batch.EDGE_COMPLETENESS_FALLBACK_CONTRACT),
            "gate": {
                "availability_non_regression": True,
                "truth_coverage_non_regression": True,
                "h_p50_max_regression_m": H_P50_MAX_REGRESSION_M,
                "h_p95_max_regression_m": H_P95_MAX_REGRESSION_M,
                "v_p95_max_regression_m": V_P95_MAX_REGRESSION_M,
                "diagnostic_max_regression_m": DIAGNOSTIC_MAX_REGRESSION_M,
            },
        },
        "truth_free_manifest": _artifact(truth_free_manifest_path),
        "routes": route_reports,
        "summary": summary,
        "truth_policy": {
            "truth_open_count": len(route_reports),
            "truth_materialized_before_truth_free_phase": False,
            "truth_used_for_candidate_selection": False,
        },
        "timing": {
            "wall_s": time.perf_counter() - started,
            "peak_rss_kb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
        },
    }
    evaluation_path = output_dir / "evaluation.json"
    batch._atomic_json(evaluation_path, evaluation)
    evaluation["artifact"] = _artifact(evaluation_path)
    # The artifact self-hash is intentionally not used by the runtime guard;
    # the externally sealed record stores the final file hash.
    batch._atomic_json(evaluation_path, evaluation)
    return evaluation


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        prog="gnss smartphone-wls-test-edge-completeness-fallback-eval"
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "output" / "smartphone-r5" / "wls-test-edge-completeness-fallback-v2" / "evaluation",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    try:
        result = run(args.output_dir)
    except (OSError, ValueError, KeyError, TypeError, wls.WlsPositionError, smoother.SmootherError) as exc:
        print(f"Edge completeness fallback evaluation failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
    print(json.dumps(result["summary"], sort_keys=True))
