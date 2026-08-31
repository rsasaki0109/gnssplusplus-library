#!/usr/bin/env python3
"""Truth-free, factor-by-factor native-FGO PDC audit.

The audit deliberately uses the already sealed development route and the
existing ``gnss_fgo`` binary.  It exercises a fixed A--F factor matrix at
1/2/30 epochs and publishes each stage atomically.  No truth file is opened.

The current native CLI exposes final positions, velocity diagnostics, factor
counts/residual RMS, and the P-factor debug rows, but it does not expose the
private clock/ambiguity state vector or no-base Doppler Jacobian rows.  This
module records those fields as unavailable instead of reconstructing them
from truth or an unverified surrogate.  The weighted Jacobian below is the
exact P-row Jacobian reconstructed from the native factor debug contract.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import resource
import subprocess
import sys
import tempfile
import time
from typing import Any, Iterable

try:
    import numpy as np
except ImportError:  # pragma: no cover - reported as a fail-closed audit error
    np = None  # type: ignore[assignment]

_COMMANDS_DIR = Path(__file__).resolve().parents[1]
if str(_COMMANDS_DIR) not in sys.path:
    sys.path.insert(0, str(_COMMANDS_DIR))
from support.gnss_runtime import application_root  # noqa: E402


ROOT = application_root(__file__)
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_gsdc2023_native_fgo_pdc_factor_audit_freeze_v1.json"
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-pdc-factor-audit-v1"
DEFAULT_BINARY = ROOT / "build/apps/gnss_fgo"
SEALED_DATASET_ID = "2021-03-16-18-59-us-ca-mtv-a/pixel5"
SEALED_ROUTE = ROOT / "output/smartphone-r5/native-fgo-v2-processed/routes/2021-03-16-18-59-us-ca-mtv-a__pixel5"
WLS_ROUTE_OUTPUT = ROOT / "output/smartphone-r5/native-fgo-pdc-wls-v1/2021-03-16-18-59-us-ca-mtv-a__pixel5/epochs-all"
SCHEMA = "smartphone-r5-gsdc2023-native-fgo-pdc-factor-audit.v1"
MAX_RUNTIME_SECONDS = 900
MAX_ADDRESS_SPACE_BYTES = 8 * 1024 * 1024 * 1024
MAX_ARTIFACT_BYTES = 1024 * 1024 * 1024
MAX_TRANSITION_SPEED_MPS = 70.0


class FactorAuditError(ValueError):
    """Raised when the sealed truth-free audit contract is not met."""


# This is intentionally a tuple of immutable recipe data.  Do not add a
# command-line candidate selector: changing it requires a new freeze record.
STAGES: tuple[dict[str, Any], ...] = (
    {
        "id": "A_P_only_1",
        "epochs": 1,
        "flags": ("--no-motion-factors", "--no-tdcp-factors"),
        "family_contract": "P",
    },
    {
        "id": "B_P_D_no_temporal_2",
        "epochs": 2,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--velocity-states",
            "--no-motion-factors",
            "--no-tdcp-factors",
        ),
        "family_contract": "P+D+velocity/clock priors",
    },
    {
        "id": "C_P_motion_2",
        "epochs": 2,
        "flags": ("--no-tdcp-factors",),
        "family_contract": "P+position/clock motion",
    },
    {
        "id": "D_P_D_velocity_motion_2",
        "epochs": 2,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
            "--no-tdcp-factors",
        ),
        "family_contract": "P+D+WLS velocity/clock priors+velocity/position/clock motion",
    },
    {
        "id": "E_P_D_TDCP_2",
        "epochs": 2,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
        ),
        "family_contract": "P+D+TDCP+motion",
    },
    {
        "id": "F_P_D_TDCP_carrier_2",
        "epochs": 2,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
            "--carrier-phase-factors",
            "--ambiguity-between-factors",
        ),
        "family_contract": "P+D+TDCP+float carrier/ambiguity-between+motion",
    },
    {
        "id": "C_P_motion_30",
        "epochs": 30,
        "flags": ("--no-tdcp-factors",),
        "family_contract": "P+position/clock motion",
    },
    {
        "id": "D_P_D_velocity_motion_30",
        "epochs": 30,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
            "--no-tdcp-factors",
        ),
        "family_contract": "P+D+WLS velocity/clock priors+velocity/position/clock motion",
    },
    {
        "id": "E_P_D_TDCP_30",
        "epochs": 30,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
        ),
        "family_contract": "P+D+TDCP+motion",
    },
    {
        "id": "F_P_D_TDCP_carrier_30",
        "epochs": 30,
        "flags": (
            "--undifferenced-doppler-factors",
            "--corrected-undifferenced-doppler-factors",
            "--doppler-velocity-wls-initialization",
            "--velocity-states",
            "--velocity-motion-factors",
            "--carrier-phase-factors",
            "--ambiguity-between-factors",
        ),
        "family_contract": "P+D+TDCP+float carrier/ambiguity-between+motion",
    },
)

BASE_FLAGS: tuple[str, ...] = (
    "--preset", "default", "--backend", "eigen", "--skip-epochs", "0",
    "--max-iterations", "8", "--relative-cost-threshold", "0",
    "--absolute-cost-threshold", "0", "--pseudorange-sigma", "3",
    "--pseudorange-elevation-power", "1", "--motion-sigma", "50",
    "--clock-motion-sigma", "300", "--velocity-prior-sigma", "100",
    "--velocity-motion-sigma", "0.01", "--position-prior-sigma", "0",
    "--clock-prior-sigma", "0", "--tdcp-sigma", "0.03",
    "--carrier-phase-sigma", "0.01", "--undifferenced-doppler-sigma", "0.2",
    "--pseudorange-huber-threshold", "4", "--carrier-phase-huber-threshold", "4",
    "--tdcp-huber-threshold", "4", "--max-tdcp-gap", "2",
    "--seed-match-tolerance", "0.01", "--seed-interpolation-max-gap", "0",
    "--tdcp-slip-threshold", "10", "--min-elevation", "10", "--min-snr", "0",
    "--min-satellites-per-epoch", "4", "--no-dd-factors", "--ionosphere-model",
    "--troposphere-model", "--quiet",
)


def sha256(path: Path) -> str:
    if not path.is_file():
        raise FactorAuditError(f"missing file: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def relative(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(ROOT.resolve()))
    except ValueError:
        return str(path)


def load_json(path: Path, label: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise FactorAuditError(f"invalid {label}: {path}") from exc
    if not isinstance(value, dict):
        raise FactorAuditError(f"{label} must be an object: {path}")
    return value


def atomic_bytes(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(content)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    atomic_bytes(path, (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"))


def artifact(path: Path) -> dict[str, Any]:
    if not path.is_file() or path.stat().st_size > MAX_ARTIFACT_BYTES:
        raise FactorAuditError(f"missing or oversized artifact: {path}")
    return {"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)}


def verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze = load_json(path, "PDC factor audit freeze")
    if freeze.get("schema_version") != "smartphone-r5-gsdc2023-native-fgo-pdc-factor-audit-freeze.v1":
        raise FactorAuditError("PDC factor audit freeze schema mismatch")
    if freeze.get("status") != "frozen-before-truth-free-factor-audit":
        raise FactorAuditError("PDC factor audit freeze is not pre-audit")
    policy = freeze.get("policy")
    if not isinstance(policy, dict):
        raise FactorAuditError("PDC factor audit policy missing")
    for key, expected in (
        ("local_only", True), ("external_mutation", False), ("token_access", False),
        ("truth_opened", 0), ("validation_opened", 0), ("holdout_opened", 0),
        ("test_data_used", False), ("leaderboard_used_for_tuning", False),
        ("production_default_changed", False), ("no_post_score_tuning", True),
    ):
        if policy.get(key) is not expected:
            raise FactorAuditError(f"PDC factor audit policy mismatch: {key}")
    route = freeze.get("sealed_route")
    if not isinstance(route, dict) or route.get("dataset_id") != SEALED_DATASET_ID:
        raise FactorAuditError("sealed route identity mismatch")
    if route.get("ground_truth_materialized_in_route") is not False:
        raise FactorAuditError("sealed route truth contract is not closed")
    if sha256(SEALED_ROUTE / "route_manifest.json") != route.get("route_manifest_sha256"):
        raise FactorAuditError("sealed route manifest hash mismatch")
    if sha256(SEALED_ROUTE / "materialization_manifest.json") != route.get("materialization_manifest_sha256"):
        raise FactorAuditError("sealed materialization manifest hash mismatch")
    for rel, expected in (route.get("members") or {}).items():
        if "truth" in rel.lower():
            raise FactorAuditError("truth-like member is forbidden")
        if sha256(SEALED_ROUTE / rel) != expected:
            raise FactorAuditError(f"sealed route member hash mismatch: {rel}")
    if (SEALED_ROUTE / "ground_truth.csv").exists() or (SEALED_ROUTE / "inputs" / "ground_truth.csv").exists():
        raise FactorAuditError("ground_truth.csv is materialized in sealed route")
    upstream = freeze.get("upstream_freeze")
    if not isinstance(upstream, dict):
        raise FactorAuditError("upstream freeze reference missing")
    upstream_path = ROOT / str(upstream.get("record", ""))
    if sha256(upstream_path) != upstream.get("sha256"):
        raise FactorAuditError("upstream WLS freeze hash mismatch")
    sources = freeze.get("source_hashes")
    if not isinstance(sources, dict):
        raise FactorAuditError("source hashes missing")
    for rel, expected in sources.items():
        if sha256(ROOT / rel) != expected:
            raise FactorAuditError(f"source/binary hash mismatch: {rel}")
    recipe = freeze.get("recipe")
    if not isinstance(recipe, dict) or recipe.get("max_iterations") != 8:
        raise FactorAuditError("fixed optimizer recipe mismatch")
    stage_matrix = recipe.get("stage_matrix")
    if not isinstance(stage_matrix, list) or len(stage_matrix) != len(STAGES):
        raise FactorAuditError("stage matrix length mismatch")
    frozen_ids = tuple(str(item.get("id")) for item in stage_matrix if isinstance(item, dict))
    if frozen_ids != tuple(str(stage["id"]) for stage in STAGES):
        raise FactorAuditError("stage matrix identity mismatch")
    return freeze


def command_for_stage(binary: Path, stage: dict[str, Any], output: Path) -> list[str]:
    return [
        str(binary), "--obs", str(SEALED_ROUTE / "adapter" / "rover.obs"),
        "--nav", str(SEALED_ROUTE / "inputs" / "brdc.nav"),
        *BASE_FLAGS, "--max-epochs", str(int(stage["epochs"])),
        "--out", str(output / "fgo.pos"),
        "--summary-json", str(output / "fgo_summary.json"),
        "--epoch-debug-csv", str(output / "fgo_epoch_debug.csv"),
        "--factor-debug-csv", str(output / "fgo_factor_debug.csv"),
        "--cost-trace-csv", str(output / "fgo_cost_trace.csv"),
        *stage["flags"],
    ]


def child_limits() -> None:
    resource.setrlimit(resource.RLIMIT_AS, (MAX_ADDRESS_SPACE_BYTES, MAX_ADDRESS_SPACE_BYTES))
    resource.setrlimit(resource.RLIMIT_CPU, (MAX_RUNTIME_SECONDS, MAX_RUNTIME_SECONDS + 1))


def run_child(command: list[str], directory: Path) -> dict[str, Any]:
    environment = os.environ.copy()
    environment["LD_LIBRARY_PATH"] = "/home/sasaki/.local/lib:/opt/ros/jazzy/lib" + (
        os.pathsep + environment["LD_LIBRARY_PATH"] if environment.get("LD_LIBRARY_PATH") else ""
    )
    started = time.perf_counter()
    return_code: int | None = None
    timed_out = False
    with (directory / "stdout.log").open("wb") as stdout, (directory / "stderr.log").open("wb") as stderr:
        try:
            completed = subprocess.run(
                command, cwd=ROOT, env=environment, stdin=subprocess.DEVNULL,
                stdout=stdout, stderr=stderr, timeout=MAX_RUNTIME_SECONDS + 5,
                preexec_fn=child_limits, check=False,
            )
            return_code = completed.returncode
        except subprocess.TimeoutExpired:
            timed_out = True
    return {
        "return_code": return_code,
        "timed_out": timed_out,
        "wall_seconds": time.perf_counter() - started,
        "child_max_rss_kib": resource.getrusage(resource.RUSAGE_CHILDREN).ru_maxrss,
    }


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    try:
        with path.open(newline="", encoding="utf-8") as handle:
            reader = csv.DictReader(handle)
            if reader.fieldnames is None:
                raise FactorAuditError(f"CSV has no header: {path}")
            return list(reader)
    except OSError as exc:
        raise FactorAuditError(f"cannot read CSV: {path}") from exc


def _summary_contract(summary: dict[str, Any], stage: dict[str, Any]) -> dict[str, Any]:
    required = (
        "input_epochs", "optimized_epochs", "valid_solutions", "pseudorange_factors",
        "undifferenced_doppler_factors", "tdcp_factors", "carrier_phase_factors",
        "ambiguity_states", "motion_factors", "graph_factors", "graph_values",
        "initial_cost", "final_cost", "iterations", "converged", "last_update_norm_m",
        "residual_rms_m", "tdcp_residual_rms_m", "undifferenced_doppler_residual_rms_mps",
        "carrier_phase_residual_rms_m", "doppler_velocity_wls_valid_epochs",
        "doppler_velocity_wls_rejected_epochs", "doppler_velocity_wls_max_velocity_norm_mps",
        "doppler_velocity_wls_max_normalized_rms",
    )
    for key in required:
        if key not in summary:
            raise FactorAuditError(f"summary field missing: {key}")
        if key != "converged" and not _finite(summary[key]):
            raise FactorAuditError(f"summary field is nonfinite: {key}")
    if summary.get("backend") != "eigen" or summary.get("preset") != "default":
        raise FactorAuditError("stage backend/preset mismatch")
    if summary.get("single_difference_doppler_factors") != 0 or summary.get("single_difference_tdcp_factors") != 0:
        raise FactorAuditError("base-dependent SD factors appeared in no-base audit")
    if int(summary["input_epochs"]) != int(stage["epochs"]) or int(summary["optimized_epochs"]) != int(stage["epochs"]):
        raise FactorAuditError("stage epoch limit was not honored")
    if int(summary["valid_solutions"]) != int(stage["epochs"]):
        raise FactorAuditError("stage did not produce one finite solution per epoch")
    if float(summary["final_cost"]) > float(summary["initial_cost"]) + 1e-6:
        raise FactorAuditError("stage final cost increased")
    return summary


def _cost_trace(path: Path, summary: dict[str, Any]) -> dict[str, Any]:
    rows = read_csv_rows(path)
    costs: list[float] = []
    for row in rows:
        value = row.get("cost")
        if not _finite(value):
            raise FactorAuditError(f"nonfinite cost trace value: {path}")
        costs.append(float(value))
    monotonic = all(right <= left + 1e-8 for left, right in zip(costs, costs[1:]))
    if costs and abs(costs[0] - float(summary["initial_cost"])) > max(1e-5, abs(costs[0]) * 1e-10):
        raise FactorAuditError("cost trace initial value disagrees with summary")
    if costs and abs(costs[-1] - float(summary["final_cost"])) > max(1e-5, abs(costs[-1]) * 1e-10):
        raise FactorAuditError("cost trace final value disagrees with summary")
    return {
        "rows": len(costs),
        "costs": costs,
        "monotonic_non_increasing": monotonic,
        "initial_cost": float(summary["initial_cost"]),
        "final_cost": float(summary["final_cost"]),
        "termination": {
            "iterations": int(summary["iterations"]),
            "converged": bool(summary["converged"]),
            "last_update_norm_m": float(summary["last_update_norm_m"]),
        },
    }


def _float_column(row: dict[str, str], key: str) -> float | None:
    value = row.get(key)
    return float(value) if _finite(value) else None


def _state_metrics(epoch_path: Path, summary: dict[str, Any]) -> dict[str, Any]:
    rows = read_csv_rows(epoch_path)
    positions: list[tuple[float, float, float]] = []
    seeds: list[tuple[float, float, float]] = []
    velocities: list[tuple[float, float, float]] = []
    times: list[float] = []
    for row in rows:
        p = tuple(_float_column(row, key) for key in ("position_x_m", "position_y_m", "position_z_m"))
        s = tuple(_float_column(row, key) for key in ("seed_position_x_m", "seed_position_y_m", "seed_position_z_m"))
        if any(value is None for value in p + s):
            raise FactorAuditError("nonfinite position/seed in epoch debug")
        positions.append(tuple(float(value) for value in p))  # type: ignore[arg-type]
        seeds.append(tuple(float(value) for value in s))  # type: ignore[arg-type]
        times.append(float(row["gps_tow"]))
        v = tuple(_float_column(row, key) for key in ("velocity_x_mps", "velocity_y_mps", "velocity_z_mps"))
        if all(value is not None for value in v):
            velocities.append(tuple(float(value) for value in v))  # type: ignore[arg-type]
    position_delta = [math.sqrt(sum((p[i] - s[i]) ** 2 for i in range(3))) for p, s in zip(positions, seeds)]
    position_norm = [math.sqrt(sum(value * value for value in p)) for p in positions]
    velocity_norm = [math.sqrt(sum(value * value for value in v)) for v in velocities]
    transition_speeds: list[float] = []
    for left, right, left_time, right_time in zip(positions, positions[1:], times, times[1:]):
        dt = right_time - left_time
        if dt > 0.0:
            transition_speeds.append(math.sqrt(sum((right[i] - left[i]) ** 2 for i in range(3))) / dt)
    return {
        "rows": len(rows),
        "position": {
            "initial_norm_m": position_norm[0] if position_norm else None,
            "final_norm_m": position_norm[-1] if position_norm else None,
            "max_norm_m": max(position_norm, default=None),
            "max_seed_delta_m": max(position_delta, default=None),
            "mean_seed_delta_m": (sum(position_delta) / len(position_delta)) if position_delta else None,
        },
        "velocity": {
            "finite_rows": len(velocities),
            "max_norm_mps": max(velocity_norm, default=None),
            "mean_norm_mps": (sum(velocity_norm) / len(velocity_norm)) if velocity_norm else None,
            "max_abs_component_mps": max((abs(value) for v in velocities for value in v), default=None),
        },
        "continuity": {
            "transition_count": len(transition_speeds),
            "max_neighbor_speed_mps": max(transition_speeds, default=None),
            "over_70mps_transition_count": sum(speed > MAX_TRANSITION_SPEED_MPS for speed in transition_speeds),
        },
        "clock_bias_m": {
            "available": False,
            "reason": "frozen gnss_fgo epoch-debug/POS contract does not expose receiver clock state",
        },
        "clock_drift_mps": {
            "available": False,
            "reason": "frozen gnss_fgo epoch-debug/POS contract does not expose receiver clock state",
        },
        "ambiguity_m": {
            "available": False,
            "state_count": int(summary["ambiguity_states"]),
            "reason": "frozen gnss_fgo output exposes ambiguity count but not final ambiguity values",
        },
    }


def _family_costs(factor_path: Path, summary: dict[str, Any]) -> dict[str, Any]:
    rows = read_csv_rows(factor_path)
    p_rows = [row for row in rows if row.get("sigma_p_m") is not None]
    initial_terms: list[float] = []
    final_terms: list[float] = []
    for row in p_rows:
        sigma = _float_column(row, "sigma_p_m")
        initial = _float_column(row, "initial_res_pc_m")
        final = _float_column(row, "final_residual_m")
        if sigma is None or sigma <= 0.0:
            raise FactorAuditError("invalid P sigma in factor debug")
        if initial is not None:
            initial_terms.append((initial / sigma) ** 2)
        if final is not None:
            final_terms.append((final / sigma) ** 2)

    def estimate(count_key: str, rms_key: str, unit: str) -> dict[str, Any]:
        count = int(summary[count_key])
        rms = float(summary[rms_key])
        return {
            "factor_count": count,
            "unit": unit,
            "final_rms": rms,
            "final_quadratic_equivalent_cost": 0.5 * count * rms * rms if count else 0.0,
            "initial_cost": None,
            "initial_cost_reason": "native summary does not expose per-family initial residuals",
        }

    result = {
        "pseudorange": estimate("pseudorange_factors", "residual_rms_m", "m"),
        "undifferenced_doppler": estimate("undifferenced_doppler_factors", "undifferenced_doppler_residual_rms_mps", "mps"),
        "tdcp": estimate("tdcp_factors", "tdcp_residual_rms_m", "m"),
        "carrier_phase": estimate("carrier_phase_factors", "carrier_phase_residual_rms_m", "m"),
        "motion": {
            "factor_count": int(summary["motion_factors"]),
            "initial_cost": None,
            "final_cost": None,
            "reason": "native summary exposes motion count but not a separate family residual",
        },
        "global": {
            "initial_cost": float(summary["initial_cost"]),
            "final_cost": float(summary["final_cost"]),
            "cost_definition": "native weighted robust graph cost",
        },
    }
    if p_rows:
        result["pseudorange"]["debug_rows"] = len(p_rows)
        result["pseudorange"]["initial_unrobust_quadratic_cost_from_debug"] = 0.5 * sum(initial_terms) if initial_terms else None
        result["pseudorange"]["final_unrobust_quadratic_cost_from_debug"] = 0.5 * sum(final_terms) if final_terms else None
    return result


def weighted_p_jacobian_metrics(factor_path: Path) -> dict[str, Any]:
    """Compute exact weighted P-row geometry metrics from native debug rows.

    Columns are epoch-local ECEF position/clock states followed by the
    inter-system clock-bias columns encountered in the dump.  The sign of the
    LOS is immaterial for rank/condition/column norms and follows the native
    emitted P Jacobian convention.
    """
    if np is None:
        return {"available": False, "reason": "numpy unavailable"}
    rows = read_csv_rows(factor_path)
    if not rows:
        return {"available": False, "reason": "factor debug is empty"}
    p_rows = [row for row in rows if all(key in row for key in ("epoch_index", "clock_group", "sigma_p_m", "los_x", "los_y", "los_z"))]
    if not p_rows:
        return {"available": False, "reason": "no native P rows in factor debug"}
    max_epoch = max(int(row["epoch_index"]) for row in p_rows)
    groups = sorted({int(row["clock_group"]) for row in p_rows if int(row["clock_group"]) != 0})
    group_column = {group: 4 * (max_epoch + 1) + index for index, group in enumerate(groups)}
    matrix = np.zeros((len(p_rows), 4 * (max_epoch + 1) + len(groups)), dtype=float)
    whitened: list[float] = []
    for index, row in enumerate(p_rows):
        epoch = int(row["epoch_index"])
        sigma = float(row["sigma_p_m"])
        if not math.isfinite(sigma) or sigma <= 0.0:
            raise FactorAuditError("invalid P sigma for Jacobian")
        offset = 4 * epoch
        matrix[index, offset:offset + 3] = [float(row[key]) / sigma for key in ("los_x", "los_y", "los_z")]
        matrix[index, offset + 3] = 1.0 / sigma
        group = int(row["clock_group"])
        if group in group_column:
            matrix[index, group_column[group]] = 1.0 / sigma
        final = _float_column(row, "final_residual_m")
        if final is not None:
            whitened.append(final / sigma)
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    finite_singular = singular_values[np.isfinite(singular_values)]
    if finite_singular.size == 0:
        raise FactorAuditError("P Jacobian SVD is nonfinite")
    largest = float(finite_singular[0])
    threshold = max(matrix.shape) * np.finfo(float).eps * max(largest, 1.0)
    rank = int(np.count_nonzero(finite_singular > threshold))
    smallest_nonzero = float(finite_singular[rank - 1]) if rank else None
    condition = largest / smallest_nonzero if smallest_nonzero and smallest_nonzero > 0.0 else math.inf
    norms = np.linalg.norm(matrix, axis=0)
    return {
        "available": True,
        "family": "pseudorange",
        "rows": int(matrix.shape[0]),
        "columns": int(matrix.shape[1]),
        "epoch_local_columns": 4 * (max_epoch + 1),
        "inter_system_clock_groups": groups,
        "rank": rank,
        "full_column_rank": rank == matrix.shape[1],
        "condition_number": condition,
        "singular_values": [float(value) for value in finite_singular],
        "column_norms": [float(value) for value in norms],
        "whitened_final_residual_rms": (
            math.sqrt(sum(value * value for value in whitened) / len(whitened)) if whitened else None
        ),
        "whitened_final_residual_rows": len(whitened),
        "definition": "native fgo_factor_debug.csv P rows; no D/TDCP/carrier surrogate",
    }


def _stage_manifest(freeze: dict[str, Any], stage: dict[str, Any], directory: Path, run: dict[str, Any], summary: dict[str, Any]) -> dict[str, Any]:
    return {
        "schema_version": SCHEMA + ".stage",
        "status": "truth-free-stage-sealed",
        "dataset_id": SEALED_DATASET_ID,
        "stage": {"id": stage["id"], "epochs": stage["epochs"], "family_contract": stage["family_contract"], "flags": list(stage["flags"])},
        "truth_opened": False,
        "validation_or_holdout_opened": False,
        "command_runtime": run,
        "summary": summary,
        "cost_trace": _cost_trace(directory / "fgo_cost_trace.csv", summary),
        "factor_family_costs": _family_costs(directory / "fgo_factor_debug.csv", summary),
        "state_metrics": _state_metrics(directory / "fgo_epoch_debug.csv", summary),
        "weighted_jacobian": weighted_p_jacobian_metrics(directory / "fgo_factor_debug.csv"),
        "artifacts": {name: artifact(directory / name) for name in (
            "fgo.pos", "fgo_summary.json", "fgo_epoch_debug.csv", "fgo_factor_debug.csv",
            "fgo_cost_trace.csv", "stdout.log", "stderr.log",
        )},
        "freeze_record": relative(FREEZE),
        "freeze_record_sha256": sha256(FREEZE),
        "source_hashes": freeze["source_hashes"],
    }


def run_audit(freeze_path: Path = FREEZE, output_root: Path = DEFAULT_OUTPUT, binary: Path = DEFAULT_BINARY) -> dict[str, Any]:
    freeze = verify_freeze(freeze_path)
    if binary.resolve() != (ROOT / "build/apps/gnss_fgo").resolve():
        raise FactorAuditError("audit binary must be the frozen Release gnss_fgo path")
    if output_root.exists():
        manifest_path = output_root / "factor_audit.manifest.json"
        audit_path = output_root / "factor_audit.json"
        if not manifest_path.is_file() or not audit_path.is_file():
            raise FactorAuditError(f"existing audit output is not sealed: {output_root}")
        return load_json(audit_path, "sealed factor audit")
    if not binary.is_file() or sha256(binary) != freeze["source_hashes"].get("build/apps/gnss_fgo"):
        raise FactorAuditError("frozen Release binary is unavailable or changed")
    output_root.parent.mkdir(parents=True, exist_ok=True)
    temporary_root = Path(tempfile.mkdtemp(prefix=f".{output_root.name}.", dir=str(output_root.parent)))
    started = time.perf_counter()
    stage_manifests: list[dict[str, Any]] = []
    try:
        for stage in STAGES:
            stage_dir = temporary_root / str(stage["id"])
            stage_dir.mkdir(parents=True, exist_ok=False)
            command = command_for_stage(binary, stage, stage_dir)
            runtime = run_child(command, stage_dir)
            if runtime["timed_out"] or runtime["return_code"] != 0:
                raise FactorAuditError(f"stage failed: {stage['id']} rc={runtime['return_code']}")
            summary = _summary_contract(load_json(stage_dir / "fgo_summary.json", "stage summary"), stage)
            stage_payload = _stage_manifest(freeze, stage, stage_dir, {"command": command, **runtime}, summary)
            atomic_json(stage_dir / "stage_manifest.json", stage_payload)
            stage_payload["stage_manifest_sha256"] = sha256(stage_dir / "stage_manifest.json")
            atomic_bytes(stage_dir / "stage_manifest.sha256", f"{stage_payload['stage_manifest_sha256']}  stage_manifest.json\n".encode("ascii"))
            stage_manifests.append(stage_payload)
        audit = {
            "schema_version": SCHEMA,
            "status": "truth-free-factor-audit-sealed",
            "dataset_id": SEALED_DATASET_ID,
            "freeze_record": relative(freeze_path),
            "freeze_record_sha256": sha256(freeze_path),
            "route_input_hashes": freeze["sealed_route"],
            "truth_policy": {"truth_opened": False, "validation_opened": False, "holdout_opened": False, "test_data_used": False},
            "runtime": {"wall_seconds": time.perf_counter() - started, "max_child_rss_kib": max((int(item["command_runtime"]["child_max_rss_kib"]) for item in stage_manifests), default=0)},
            "stages": stage_manifests,
            "first_structural_failure": None,
            "parity_oracle": freeze["parity_oracle"],
            "limitations": [
                "No new truth score was read; this is structural only.",
                "Clock bias/drift and final ambiguity values are unavailable in the frozen CLI output contract and are recorded null rather than inferred.",
                "The checkout has no sealed taroz dogfood factor artifact; exact numeric parity is therefore unavailable and no surrogate comparison is claimed.",
                "No production default, v5 artifact, candidate parameter, or algorithm was changed.",
            ],
        }
        atomic_json(temporary_root / "factor_audit.json", audit)
        audit_hash = sha256(temporary_root / "factor_audit.json")
        manifest = {
            "schema_version": SCHEMA + ".manifest",
            "audit": "factor_audit.json",
            "audit_sha256": audit_hash,
            "freeze_record": relative(freeze_path),
            "freeze_record_sha256": sha256(freeze_path),
            "stage_count": len(stage_manifests),
            "truth_opened": False,
            "artifacts": {"factor_audit.json": {"bytes": (temporary_root / "factor_audit.json").stat().st_size, "sha256": audit_hash}},
        }
        atomic_json(temporary_root / "factor_audit.manifest.json", manifest)
        atomic_bytes(temporary_root / "factor_audit.manifest.sha256", f"{sha256(temporary_root / 'factor_audit.manifest.json')}  factor_audit.manifest.json\n".encode("ascii"))
        os.replace(temporary_root, output_root)
        return audit
    except Exception:
        # The incomplete temporary directory is deliberately left for forensic
        # inspection; it is outside the published content-addressed root.
        raise


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--freeze-record", type=Path, default=FREEZE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--binary", type=Path, default=DEFAULT_BINARY)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        audit = run_audit(args.freeze_record, args.output_root, args.binary)
    except FactorAuditError as exc:
        print(f"factor audit failed closed: {exc}", file=sys.stderr)
        return 2
    print(json.dumps({"status": audit["status"], "stages": len(audit["stages"]), "truth_opened": audit["truth_policy"]["truth_opened"]}, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
