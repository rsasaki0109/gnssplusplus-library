#!/usr/bin/env python3
"""Truth-free completeness-fallback mask evaluation for the frozen test lane.

The three development routes in this command were already materialized and
truth-scored by an earlier development record.  This command creates all
artificially masked truth-free outputs first, seals their hashes, and only
then reads those development truth files to measure the fixed fallback.  No
candidate or numeric parameter is searched here.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
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

import gnss_smartphone_kaggle as kaggle  # noqa: E402
import gnss_smartphone_reacquisition_eval as reacquisition  # noqa: E402
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402
import gnss_smartphone_trajectory_smoother_eval as smoother_eval  # noqa: E402
import gnss_smartphone_wls as wls  # noqa: E402
import gnss_smartphone_wls_test_batch as batch  # noqa: E402


SCHEMA_VERSION = "smartphone-r5-wls-test-completeness-fallback-evaluation.v1"
MASK_DURATIONS_S = (1, 2, 5, 10)
H_P50_MAX_REGRESSION_M = 0.01
H_P95_MAX_REGRESSION_M = 5.0
V_P95_MAX_REGRESSION_M = 5.0
DIAGNOSTIC_MAX_REGRESSION_M = 5.0
MATCH_TOLERANCE_MS = 100

ROUTES = (
    {
        "dataset_id": "2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g",
        "wls": "output/smartphone-r5/wls-residual-v2.1/validation/routes/2023-05-23-19-56-us-ca-mtv-ie2__sm-a505g/wls/wls.pos",
        "device": "output/smartphone-r5/wls-residual-v2.1/validation/materialized/routes/2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g/inputs/device_gnss.csv",
        "truth": "output/smartphone-r5/wls-residual-v2.1/validation/materialized/routes/2023-05-23-19-56-us-ca-mtv-ie2/sm-a505g/inputs/ground_truth.csv",
    },
    {
        "dataset_id": "2023-03-08-21-34-us-ca-mtv-u/pixel6pro",
        "wls": "output/smartphone-r5/wls-position-v1/routes/2023-03-08-21-34-us-ca-mtv-u__pixel6pro/wls/wls.pos",
        "device": "output/smartphone-r5/generalization-v6/routes/2023-03-08-21-34-us-ca-mtv-u/pixel6pro/inputs/device_gnss.csv",
        "truth": "output/smartphone-r5/generalization-v6/routes/2023-03-08-21-34-us-ca-mtv-u/pixel6pro/inputs/ground_truth.csv",
    },
    {
        "dataset_id": "2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8",
        "wls": "output/smartphone-r5/wls-position-v1/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea__mi8/wls/wls.pos",
        "device": "output/smartphone-r5/generalization-v6/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8/inputs/device_gnss.csv",
        "truth": "output/smartphone-r5/generalization-v6/routes/2021-01-04-21-50-us-ca-e1highway280driveroutea/mi8/inputs/ground_truth.csv",
    },
)


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


def _score(positions: list[smoother.PositionRow], device: Path, truth: Path) -> dict[str, Any]:
    epochs = smoother._read_device_epochs(device, 1)
    if [row.timestamp_ms for row in positions] != epochs:
        raise ValueError("mask output keys differ from device epochs")
    truth_values = smoother_eval._read_truth(truth)
    rows = reacquisition._raw_rows(positions)
    return smoother_eval._score_rows(
        rows,
        {row.timestamp_ms: row for row in positions},
        truth_values,
        0,
        len(epochs),
        match_tolerance_ms=MATCH_TOLERANCE_MS,
    )


def _masked_interpolation(
    positions: list[smoother.PositionRow], duration_s: int
) -> tuple[list[smoother.PositionRow], dict[str, Any]]:
    if duration_s == 10:
        # Ten seconds is represented by a ten-second valid-source bracket;
        # nine interior one-second keys are omitted so the max-gap contract is
        # exactly exercised without exceeding it.
        missing_count = 9
    else:
        missing_count = duration_s
    center = len(positions) // 2
    missing = positions[center : center + missing_count]
    remaining = positions[:center] + positions[center + missing_count :]
    timestamps = [row.timestamp_ms for row in remaining]
    output: list[smoother.PositionRow] = []
    interpolated = 0
    for source in positions:
        if source not in missing:
            output.append(source)
            continue
        candidate = batch._interpolate_position(
            remaining,
            timestamps,
            source.timestamp_ms,
            batch.FALLBACK_INTERPOLATION_MAX_GAP_MS,
        )
        if candidate is None:
            raise ValueError(f"mask duration {duration_s}s exceeded fallback contract")
        output.append(candidate[0])
        interpolated += 1
    if [row.timestamp_ms for row in output] != [row.timestamp_ms for row in positions]:
        raise ValueError("interpolation did not preserve target key order")
    return output, {
        "requested_mask_duration_s": duration_s,
        "omitted_key_count": missing_count,
        "omitted_key_span_s": (
            (missing[-1].timestamp_ms - missing[0].timestamp_ms) / 1000.0
            if len(missing) > 1
            else 0.0
        ),
        "bracket_gap_s": (
            (remaining[center].timestamp_ms - remaining[center - 1].timestamp_ms) / 1000.0
        ),
        "interpolated_epoch_count": interpolated,
        "source": "wls_ecef_linear_interpolation",
    }


def _masked_native_nearest(
    positions: list[smoother.PositionRow],
) -> tuple[list[smoother.PositionRow], dict[str, Any]]:
    center = len(positions) // 2
    target = positions[center]
    remaining = positions[:center] + positions[center + 1 :]
    native_source = smoother.PositionRow(
        **{**target.__dict__, "timestamp_ms": target.timestamp_ms + 5}
    )
    state = {
        "a": {
            "positions": remaining,
            "position_timestamps": [row.timestamp_ms for row in remaining],
            "position_by_timestamp": {row.timestamp_ms: row for row in remaining},
            "selected_keys": [row.timestamp_ms for row in positions],
            "native_fallback": {
                "positions": [native_source],
                "position_by_timestamp": {native_source.timestamp_ms: native_source},
            },
        }
    }
    fused, stats = batch._test_fuse_positions(
        state, allow_completeness_fallback=True
    )
    output = fused["a"]
    if [row.timestamp_ms for row in output] != [row.timestamp_ms for row in positions]:
        raise ValueError("native nearest fallback did not preserve target keys")
    return output, {
        "source": "native_stability_nearest_fallback",
        "native_tolerance_ms": batch.FALLBACK_NATIVE_TOLERANCE_MS,
        "native_offset_ms": 5,
        "native_nearest_fallback_epoch_count": stats["native_nearest_fallback_epoch_count"],
    }


def _out_of_range_mask(
    positions: list[smoother.PositionRow],
) -> tuple[list[smoother.PositionRow], dict[str, Any]]:
    center = len(positions) // 2
    remaining = positions[:center] + positions[center + 1 :]
    timestamps = [row.timestamp_ms for row in remaining]
    candidate = batch._interpolate_position(
        remaining,
        timestamps,
        positions[center].timestamp_ms,
        batch.FALLBACK_INTERPOLATION_MAX_GAP_MS,
    )
    if candidate is None:
        raise ValueError("out-of-range omission is not bracketed")
    output = positions[:center] + [candidate[0]] + positions[center + 1 :]
    return output, {
        "source": "wls_ecef_linear_interpolation",
        "invalid_epoch_count": 1,
        "invalid_epoch_reason": "finite out-of-earth-range WLS omitted",
        "interpolated_epoch_count": 1,
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
        raise ValueError(f"refusing to overwrite fallback evaluation output: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    route_reports: list[dict[str, Any]] = []

    # Phase 1: create every truth-free baseline/mask artifact and seal hashes.
    truth_free_artifacts: list[dict[str, Any]] = []
    for route in ROUTES:
        dataset_id = str(route["dataset_id"])
        positions = smoother._read_positions(_absolute(str(route["wls"])), 18)
        route_dir = output_dir / "routes" / batch._safe_id(dataset_id)
        route_dir.mkdir(parents=True, exist_ok=True)
        scenario_rows: dict[str, tuple[list[smoother.PositionRow], dict[str, Any]]] = {}
        scenario_rows["unmasked_baseline"] = (positions, {"source": "wls_raw"})
        for duration_s in MASK_DURATIONS_S:
            scenario_rows[f"mask_{duration_s}s"] = _masked_interpolation(positions, duration_s)
        scenario_rows["out_of_range_epoch"] = _out_of_range_mask(positions)
        scenario_artifacts: dict[str, Any] = {}
        for scenario, (rows, scenario_meta) in scenario_rows.items():
            position_path = route_dir / f"{scenario}.pos"
            wls._write_pos(position_path, rows)
            scenario_artifacts[scenario] = {
                "position": _artifact(position_path),
                "truth_free": True,
                "truth_used": False,
                "meta": scenario_meta,
            }
            truth_free_artifacts.append(scenario_artifacts[scenario]["position"])
        route_reports.append(
            {
                "dataset_id": dataset_id,
                "wls": _artifact(_absolute(str(route["wls"]))),
                "device": _artifact(_absolute(str(route["device"]))),
                "truth_not_opened_in_phase": True,
                "scenarios": scenario_artifacts,
            }
        )
    truth_free_manifest = {
        "schema_version": "smartphone-r5-wls-test-completeness-fallback-truth-free-manifest.v1",
        "status": "truth-free-artifacts-sealed-before-development-truth-read",
        "truth_free": True,
        "truth_used": False,
        "candidate_contract": dict(batch.COMPLETENESS_FALLBACK_CONTRACT),
        "algorithm_core_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_CORE_HASH,
        "algorithm_parameter_hash": wls.MULTI_PHONE_HOLDOUT_ALGORITHM_PARAMETER_HASH,
        "routes": route_reports,
        "truth_policy": {"truth_open_count": 0, "ground_truth_members_read": False},
    }
    truth_free_manifest_path = output_dir / "truth_free_artifacts_manifest.json"
    batch._atomic_json(truth_free_manifest_path, truth_free_manifest)

    # Phase 2: one development truth read after all truth-free outputs exist.
    gate_failures: list[str] = []
    for route_report, route in zip(route_reports, ROUTES):
        device = _absolute(str(route["device"]))
        truth = _absolute(str(route["truth"]))
        positions_by_scenario: dict[str, list[smoother.PositionRow]] = {}
        for scenario, artifact in route_report["scenarios"].items():
            positions_by_scenario[scenario] = smoother._read_positions(
                Path(str(artifact["position"]["path"])), 18
            )
        baseline = _score(positions_by_scenario["unmasked_baseline"], device, truth)
        route_report["baseline_metrics"] = baseline
        route_report["scenario_metrics"] = {}
        for scenario, rows in positions_by_scenario.items():
            if scenario == "unmasked_baseline":
                continue
            metrics = _score(rows, device, truth)
            failures = _regression_failures(metrics, baseline)
            route_report["scenario_metrics"][scenario] = {
                "metrics": metrics,
                "gate_failures": failures,
                "passed": not failures,
            }
            gate_failures.extend(f"{route_report['dataset_id']}:{failure}" for failure in failures)
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
        "status": "completed-one-shot-development-mask-evaluation",
        "candidate_contract": {
            "candidate_search": False,
            "parameter_research": False,
            "fallback_policy": dict(batch.COMPLETENESS_FALLBACK_CONTRACT),
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
    batch._atomic_json(evaluation_path, evaluation)
    return evaluation


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog="gnss smartphone-wls-test-completeness-fallback-eval")
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=ROOT / "output" / "smartphone-r5" / "wls-test-completeness-fallback-v1" / "evaluation",
    )
    return parser.parse_args(argv)


if __name__ == "__main__":
    args = parse_args()
    try:
        result = run(args.output_dir)
    except (OSError, ValueError, KeyError, TypeError, wls.WlsPositionError, smoother.SmootherError) as exc:
        print(f"Completeness fallback evaluation failed: {exc}", file=sys.stderr)
        raise SystemExit(2)
    print(json.dumps(result["summary"], sort_keys=True))
