#!/usr/bin/env python3
"""Truth-free Phase57 audit of Android rate uncertainty.

The command reads the four frozen Pixel5 ``device_gnss.csv`` files exactly
once in one process.  It reconstructs Phase25 raw pseudorange, forms the
Phase41-sign-consistent code/rate closure residual, removes only same-epoch
receiver common mode, and audits ``PseudorangeRateUncertaintyMetersPerSecond``
against that residual.  It also computes a raw-only lower-bound impact proxy
against the current fixed FGO Doppler sigma and the source-exact SNR-derived
sigma path.  No truth, navigation, solver, trajectory, coordinate, IMU,
MAT, validation, holdout, archive, or prior metric payload is read.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARKS) not in sys.path:
    sys.path.insert(0, str(BENCHMARKS))

# Phase48 supplies only the already-reviewed raw CSV parser and Phase25
# pseudorange/segment implementation.  No Phase48 output or metric payload is
# imported or opened.
import gnss_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit as phase48  # noqa: E402


FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase57_pixel5_rate_uncertainty_freeze_v1.json"
FREEZE_SHA256 = "11501f4a03dd03cefecb3ff931a9ce8a1b4322953daacd84a95e62c19397fafd"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase57_pixel5_rate_uncertainty_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase57-pixel5-rate-uncertainty-v1"

SCHEMA = "smartphone-r5-phase57-pixel5-rate-uncertainty.v1"
FREEZE_SCHEMA = "smartphone-r5-phase57-pixel5-rate-uncertainty-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase57-pixel5-rate-uncertainty-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = 299_792_458.0
TIME_GAP_NS = 1_000_000_000
TRANSITION_MAX_NS = 1_500_000_000
MIN_P = 10_000_000.0
MAX_P = 40_000_000.0
FIXED_SIGMA_MPS = 0.2
SINGLE_DIFFERENCE_SIGMA_MPS = math.hypot(FIXED_SIGMA_MPS, FIXED_SIGMA_MPS)

FIXED_BINS = ("<=0.1mps", "0.1_to_0.5mps", "0.5_to_2mps", ">2mps")
LOG_BINS = ("<0.03mps", "0.03_to_0.1mps", "0.1_to_0.3mps", "0.3_to_1mps", "1_to_3mps", ">3mps")
HIGH_BIN = ">0.5mps"
LOW_BIN = "<=0.1mps"


class Phase57Error(ValueError):
    """Raised when the immutable Phase57 audit contract is violated."""


def _fail(message: str) -> Phase57Error:
    return Phase57Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "kaggle",
        "token", "archive", "device_wls", "precomputed", "svposition",
        "svelevation", "device_imu", "coordinates",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase57 path: {path}")


def _sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _read_bytes_once(
    path: Path,
    label: str,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> tuple[bytes, str]:
    _reject_path(path)
    if not path.is_file():
        raise _fail(f"missing {label}: {path}")
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise _fail(f"failed to read {label}: {exc}") from exc
    if expected_bytes is not None and len(payload) != expected_bytes:
        raise _fail(f"{label} byte count mismatch: {len(payload)} != {expected_bytes}")
    digest = _sha256_bytes(payload)
    if expected_sha256 is not None and digest != expected_sha256:
        raise _fail(f"{label} hash mismatch: {digest} != {expected_sha256}")
    return payload, digest


def _load_json_once(
    path: Path,
    label: str,
    expected_sha256: str | None = None,
    expected_bytes: int | None = None,
) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256, expected_bytes)
    try:
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be an object")
    return value, digest


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
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


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes(value)
    _atomic_write(path, payload)
    return payload


def _is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _median(values: Iterable[float]) -> float:
    data = [float(value) for value in values]
    return float(statistics.median(data)) if data else 0.0


def _percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(float(value) for value in values)
    if not data:
        return 0.0
    rank = fraction * (len(data) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    return data[lower] + (rank - lower) * (data[upper] - data[lower])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values]
    if not data:
        return 0.0
    actual = _median(data) if center is None else float(center)
    return _median(abs(value - actual) for value in data)


def _distribution(values: Iterable[float], center: float = 0.0) -> dict[str, Any]:
    data = [float(value) for value in values]
    if not data:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    absolute = [abs(value - center) for value in data]
    return {
        "count": len(data),
        "median": _median(data),
        "mad": _mad(data, center),
        "p50_abs": _percentile(absolute, 0.50),
        "p95_abs": _percentile(absolute, 0.95),
        "max_abs": max(absolute),
    }


def _rank(values: Sequence[float]) -> list[float]:
    ordered = sorted(enumerate(values), key=lambda item: item[1])
    ranks = [0.0] * len(values)
    index = 0
    while index < len(ordered):
        end = index + 1
        while end < len(ordered) and ordered[end][1] == ordered[index][1]:
            end += 1
        rank = (index + end - 1) / 2.0 + 1.0
        for position in range(index, end):
            ranks[ordered[position][0]] = rank
        index = end
    return ranks


def _spearman(x: Sequence[float], y: Sequence[float]) -> float:
    if len(x) != len(y) or len(x) < 2:
        return 0.0
    rx, ry = _rank(x), _rank(y)
    mx, my = statistics.fmean(rx), statistics.fmean(ry)
    den_x = math.sqrt(sum((value - mx) ** 2 for value in rx))
    den_y = math.sqrt(sum((value - my) ** 2 for value in ry))
    if den_x == 0.0 or den_y == 0.0:
        return 0.0
    return sum((a - mx) * (b - my) for a, b in zip(rx, ry)) / (den_x * den_y)


def _linear_percentile(values: Iterable[float], percentile: float) -> float:
    """Source-exact MATLAB prctile used by upstream SNR sigma."""
    data = sorted(float(value) for value in values if _is_finite(value))
    if not data or not 0.0 <= percentile <= 100.0:
        return math.nan
    if len(data) == 1:
        return data[0]
    rank = 0.5 + percentile / 100.0 * len(data)
    if rank <= 1.0:
        return data[0]
    if rank >= len(data):
        return data[-1]
    lower_rank = math.floor(rank)
    lower = int(lower_rank - 1)
    return data[lower] + (rank - lower_rank) * (data[lower + 1] - data[lower])


def _signal_band(signal: str) -> str | None:
    if signal in {"GPS_L1CA", "GLO_G1CA", "GAL_E1", "BDS_B1I", "BDS_B1C"}:
        return "L1"
    if signal in {"GPS_L5", "GAL_E5A", "BDS_B2A"}:
        return "L5"
    return None


def _snr_doppler_sigma(signal: str, cn0: float | None, p85: dict[str, float]) -> float | None:
    band = _signal_band(signal)
    if band is None or cn0 is None or not _is_finite(cn0) or not _is_finite(p85.get(band)):
        return None
    scale = 10.0 ** (-(float(cn0) - float(p85[band])) / 20.0)
    sigma = scale / 12.0
    return sigma if _is_finite(sigma) and sigma > 0.0 else None


def _fixed_bin(value: float | None) -> str:
    if value is None or not _is_finite(value) or value <= 0.0:
        return "missing"
    if value <= 0.1:
        return "<=0.1mps"
    if value <= 0.5:
        return "0.1_to_0.5mps"
    if value <= 2.0:
        return "0.5_to_2mps"
    return ">2mps"


def _log_bin(value: float | None) -> str:
    if value is None or not _is_finite(value) or value <= 0.0:
        return "missing"
    if value < 0.03:
        return "<0.03mps"
    if value < 0.1:
        return "0.03_to_0.1mps"
    if value < 0.3:
        return "0.1_to_0.3mps"
    if value < 1.0:
        return "0.3_to_1mps"
    if value <= 3.0:
        return "1_to_3mps"
    return ">3mps"


def _supported(row: Any) -> bool:
    return row.system in {"GPS", "GLONASS", "GALILEO", "BEIDOU"} and row.signal in set(phase48.SIGNAL_MAP.values())


def _transition_items(epochs: Sequence[Any]) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    by_key: dict[tuple[str, int, str], list[Any]] = defaultdict(list)
    for epoch in epochs:
        for row in epoch.rows:
            by_key[row.sat_signal].append(row)
    transitions: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for key, rows in sorted(by_key.items()):
        rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        for previous, current in zip(rows, rows[1:]):
            dt_ns = current.time_ns - previous.time_ns
            if dt_ns <= 0 or dt_ns > TRANSITION_MAX_NS:
                continue
            if current.hcdc != previous.hcdc or current.segment != previous.segment:
                continue
            if not _supported(previous) or not _supported(current):
                continue
            if previous.rate_mps is None or current.rate_mps is None:
                continue
            if previous.pseudorange_m is None or current.pseudorange_m is None:
                continue
            if previous.code_masked or current.code_masked:
                continue
            dt = dt_ns / 1.0e9
            raw = (current.pseudorange_m - previous.pseudorange_m) - 0.5 * (
                float(previous.rate_mps) + float(current.rate_mps)
            ) * dt
            if not _is_finite(raw):
                continue
            previous_u = (float(previous.rate_uncertainty_mps)
                          if previous.rate_uncertainty_mps is not None and _is_finite(previous.rate_uncertainty_mps)
                          and float(previous.rate_uncertainty_mps) > 0.0 else None)
            current_u = (float(current.rate_uncertainty_mps)
                         if current.rate_uncertainty_mps is not None and _is_finite(current.rate_uncertainty_mps)
                         and float(current.rate_uncertainty_mps) > 0.0 else None)
            pair_u = math.hypot(previous_u, current_u) if previous_u is not None and current_u is not None else None
            integrated = 0.5 * dt * pair_u if pair_u is not None else None
            transitions.append({
                "utcTimeMillis": int(current.utc_ms),
                "previous_utcTimeMillis": int(previous.utc_ms),
                "dt_s": dt,
                "system": current.system,
                "svid": int(current.svid),
                "signal": current.signal,
                "hcdc": int(current.hcdc),
                "segment": int(current.segment),
                "state": int(current.state) if current.state is not None else None,
                "multipath": int(current.multipath) if current.multipath is not None else None,
                "cn0": float(current.cn0) if current.cn0 is not None and _is_finite(current.cn0) else None,
                "raw_residual_m": float(raw),
                "previous_rate_uncertainty_mps": previous_u,
                "rate_uncertainty_mps": current_u,
                "pair_uncertainty_mps": pair_u,
                "pair_integrated_sigma_m": integrated,
            })
    by_clock: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in transitions:
        by_clock[(item["utcTimeMillis"], item["hcdc"])].append(item["raw_residual_m"])
    for item in transitions:
        center = _median(by_clock[(item["utcTimeMillis"], item["hcdc"])])
        item["clock_group_median_m"] = center
        item["centered_residual_m"] = item["raw_residual_m"] - center
        item["abs_centered_residual_m"] = abs(item["centered_residual_m"])
        if item["abs_centered_residual_m"] >= 1.0:
            events.append({
                "kind": "code_rate_residual_outlier",
                "utcTimeMillis": item["utcTimeMillis"],
                "system": item["system"],
                "svid": item["svid"],
                "signal": item["signal"],
                "abs_centered_residual_m": item["abs_centered_residual_m"],
            })
    return transitions, events


def _relation_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    fixed: dict[str, list[float]] = {name: [] for name in ("missing",) + FIXED_BINS}
    logs: dict[str, list[float]] = {name: [] for name in ("missing",) + LOG_BINS}
    present: list[dict[str, Any]] = []
    normalized: list[float] = []
    for item in items:
        uncertainty = item.get("pair_uncertainty_mps")
        residual = float(item["abs_centered_residual_m"])
        fixed[_fixed_bin(uncertainty)].append(residual)
        logs[_log_bin(uncertainty)].append(residual)
        if uncertainty is not None and _is_finite(uncertainty) and uncertainty > 0.0:
            present.append(item)
            sigma = item.get("pair_integrated_sigma_m")
            item["normalized_abs_centered_residual"] = residual / float(sigma) if sigma and _is_finite(sigma) and sigma > 0.0 else None
            if item["normalized_abs_centered_residual"] is not None:
                normalized.append(float(item["normalized_abs_centered_residual"]))
        else:
            item["normalized_abs_centered_residual"] = None

    def summarize(mapping: dict[str, list[float]]) -> dict[str, Any]:
        return {
            name: {
                "count": len(values),
                "median_m": _median(values),
                "p95_m": _percentile(values, 0.95),
                "max_m": max(values) if values else 0.0,
            }
            for name, values in mapping.items()
        }

    pair_values = [float(item["pair_uncertainty_mps"]) for item in present]
    residual_values = [float(item["abs_centered_residual_m"]) for item in present]
    low = fixed[LOW_BIN]
    high = fixed["0.5_to_2mps"] + fixed[">2mps"]
    low_p95 = _percentile(low, 0.95)
    high_p95 = _percentile(high, 0.95)
    ratio = high_p95 / low_p95 if low_p95 > 0.0 and high else None
    medians = [summarize(fixed)[name]["median_m"] for name in FIXED_BINS if fixed[name]]
    p95s = [summarize(fixed)[name]["p95_m"] for name in FIXED_BINS if fixed[name]]
    normalized_le_one = sum(value <= 1.0 for value in normalized) / len(normalized) if normalized else 0.0
    return {
        "fixed_bins": summarize(fixed),
        "log_bins": summarize(logs),
        "pair_uncertainty_quartiles_mps": {
            "q25": _percentile(pair_values, 0.25),
            "q50": _percentile(pair_values, 0.50),
            "q75": _percentile(pair_values, 0.75),
            "count": len(pair_values),
        },
        "normalized_abs_centered_residual": _distribution(normalized, 0.0),
        "normalized_le_one_fraction": normalized_le_one,
        "spearman_pair_uncertainty_abs_centered_residual": _spearman(pair_values, residual_values),
        "high_low": {
            "high_definition": HIGH_BIN,
            "low_definition": LOW_BIN,
            "low_count": len(low),
            "high_count": len(high),
            "low_p95_m": low_p95,
            "high_p95_m": high_p95,
            "p95_excess_m": high_p95 - low_p95 if high else None,
            "p95_ratio": ratio,
        },
        "transition_count": len(items),
        "finite_positive_uncertainty_count": len(present),
        "populated_fixed_bins": sum(1 for name in FIXED_BINS if fixed[name]),
        "fixed_bin_median_non_decreasing": all(left <= right for left, right in zip(medians, medians[1:])),
        "fixed_bin_p95_non_decreasing": all(left <= right for left, right in zip(p95s, p95s[1:])),
    }


def _group_summary(items: Sequence[dict[str, Any]], key_fn: Any) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in items:
        groups[str(key_fn(item))].append(item)
    result: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        values = [float(item["abs_centered_residual_m"]) for item in group]
        uncertainty = [float(item["pair_uncertainty_mps"]) for item in group if item.get("pair_uncertainty_mps") is not None]
        residual = [float(item["abs_centered_residual_m"]) for item in group if item.get("pair_uncertainty_mps") is not None]
        result[name] = {
            "count": len(group),
            "uncertainty_count": len(uncertainty),
            "satellites": sorted({int(item["svid"]) for item in group}),
            "residual_distribution_m": _distribution(values, 0.0),
            "uncertainty_median_mps": _median(uncertainty),
            "spearman": _spearman(uncertainty, residual),
        }
    return result


def _snr_percentiles(epochs: Sequence[Any]) -> dict[str, float]:
    values: dict[str, list[float]] = {"L1": [], "L5": []}
    for epoch in epochs:
        for row in epoch.rows:
            if not _supported(row) or row.cn0 is None or not _is_finite(row.cn0):
                continue
            band = _signal_band(row.signal)
            if band:
                values[band].append(float(row.cn0))
    return {band: _linear_percentile(rows, 85.0) for band, rows in values.items()}


def _factor_impact(items: Sequence[dict[str, Any]], snr_p85: dict[str, float]) -> dict[str, Any]:
    fixed_rows = [item for item in items if item.get("rate_uncertainty_mps") is not None]
    fixed_affected = [item for item in fixed_rows if float(item["rate_uncertainty_mps"]) > FIXED_SIGMA_MPS]
    snr_items: list[dict[str, Any]] = []
    snr_affected: list[dict[str, Any]] = []
    for item in fixed_rows:
        sigma = _snr_doppler_sigma(item["signal"], item.get("cn0"), snr_p85)
        item["snr_enabled_existing_sigma_mps"] = sigma
        if sigma is not None:
            snr_items.append(item)
            if float(item["rate_uncertainty_mps"]) > sigma:
                snr_affected.append(item)
    inflation = [max(0.0, float(item["rate_uncertainty_mps"]) - FIXED_SIGMA_MPS) for item in fixed_rows]
    return {
        "raw_eligible_factor_proxy_count": len(items),
        "finite_positive_uncertainty_factor_proxy_count": len(fixed_rows),
        "finite_positive_uncertainty_fraction": len(fixed_rows) / len(items) if items else 0.0,
        "fixed_configured_sigma_mps": FIXED_SIGMA_MPS,
        "single_difference_pair_sigma_lower_bound_mps": SINGLE_DIFFERENCE_SIGMA_MPS,
        "fixed_lower_bound_affected_count": len(fixed_affected),
        "fixed_lower_bound_affected_fraction": len(fixed_affected) / len(items) if items else 0.0,
        "snr_enabled_sigma_available_count": len(snr_items),
        "snr_enabled_sigma_available_fraction": len(snr_items) / len(items) if items else 0.0,
        "snr_enabled_affected_count": len(snr_affected),
        "snr_enabled_affected_fraction": len(snr_affected) / len(items) if items else 0.0,
        "snr_percentile85_dbhz": snr_p85,
        "fixed_lower_bound_sigma_inflation_proxy_1s_m": {
            "median": _median(inflation),
            "p95": _percentile(inflation, 0.95),
            "max": max(inflation) if inflation else 0.0,
        },
    }


def _route_report(route: str, epochs: Sequence[Any], metadata: dict[str, Any], path: Path, digest: str) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, boundary_events = phase48._assign_segments(epochs)
    transitions, residual_events = _transition_items(epochs)
    relation = _relation_summary(transitions)
    snr_p85 = _snr_percentiles(epochs)
    impact = _factor_impact(transitions, snr_p85)
    residual_values = [float(item["abs_centered_residual_m"]) for item in transitions]
    raw_values = [float(item["raw_residual_m"]) for item in transitions]
    uncertainty_values = [float(item["rate_uncertainty_mps"]) for item in transitions if item.get("rate_uncertainty_mps") is not None]
    satellites = sorted({(item["system"], int(item["svid"])) for item in transitions})
    state_groups = _group_summary(transitions, lambda item: "missing" if item.get("state") is None else str(item["state"]))
    signal_groups = _group_summary(transitions, lambda item: f"{item['system']}:{item['signal']}")
    satellite_groups = _group_summary(transitions, lambda item: f"{item['system']}:{item['svid']}")
    raw_abs = [abs(value) for value in raw_values]
    rows = {
        "raw": int(metadata["raw_rows"]),
        "non_raw": int(metadata["non_raw_rows"]),
        "epochs": len(epochs),
        "transition_count": len(transitions),
        "transition_satellite_count": len(satellites),
        "transition_svids": [f"{system}:{svid}" for system, svid in satellites],
        "existing_mask_retention": len(transitions) / int(metadata["raw_rows"]) if metadata["raw_rows"] else 0.0,
        "uncertainty_present_fraction": len(uncertainty_values) / len(transitions) if transitions else 0.0,
        "repeated_epoch_key_count": int(metadata["repeated_epoch_key_count"]),
        "nonmonotonic_epoch_key_count": int(metadata["nonmonotonic_epoch_key_count"]),
        "unsupported_signal_rows_informational": int(metadata["unsupported_signal_rows"]),
    }
    gate_observations = {
        "all_finite_residuals": all(_is_finite(value) for value in residual_values),
        "all_finite_uncertainties": all(_is_finite(value) for value in uncertainty_values),
        "transition_count": len(transitions),
        "satellite_count": len(satellites),
        "finite_positive_uncertainty_fraction": relation["finite_positive_uncertainty_count"] / len(transitions) if transitions else 0.0,
        "populated_fixed_bins": relation["populated_fixed_bins"],
        "spearman": relation["spearman_pair_uncertainty_abs_centered_residual"],
        "high_low_excess_m": relation["high_low"]["p95_excess_m"],
        "high_low_ratio": relation["high_low"]["p95_ratio"],
        "normalized_median": relation["normalized_abs_centered_residual"]["median"],
        "normalized_le_one_fraction": relation["normalized_le_one_fraction"],
        "noncommon_centered_p95_m": _percentile(residual_values, 0.95),
        "raw_median_abs_m": _median(raw_abs),
        "centered_median_abs_m": _median(residual_values),
        "centered_to_raw_median_ratio": _median(residual_values) / _median(raw_abs) if _median(raw_abs) > 0.0 else 0.0,
        "fixed_lower_bound_affected_fraction": impact["fixed_lower_bound_affected_fraction"],
        "snr_enabled_affected_fraction": impact["snr_enabled_affected_fraction"],
        "displacement_potential_proxy_p95_m": impact["fixed_lower_bound_sigma_inflation_proxy_1s_m"]["p95"],
    }
    report = {
        "route": route,
        "input": {"path": _relative(path), "bytes": int(path.stat().st_size), "sha256": digest},
        "headers": {
            "required_columns": list(phase48.REQUIRED_COLUMNS),
            "candidate_field": "PseudorangeRateUncertaintyMetersPerSecond" in metadata["header_columns"],
            "columns": metadata["header_columns"],
        },
        "rows": rows,
        "raw_code_rate_residual": {
            "raw_distribution_m": _distribution(raw_values, 0.0),
            "clock_centered_abs_distribution_m": _distribution(residual_values, 0.0),
            "noncommon_centering": "subtract endpoint utcTimeMillis+HCDC median across eligible supported satellites/signals",
        },
        "rate_uncertainty": {
            "units": "meters per second",
            "distribution_mps": _distribution(uncertainty_values, 0.0),
            "relation": relation,
        },
        "fgo_adoption_proxy": impact,
        "snr_source_sigma": {
            "percentile": 85.0,
            "p85_dbhz_by_band": snr_p85,
            "formula": "10^(-(Cn0DbHz-p85_band)/20)/12",
            "enabled_only_as_raw_counterfactual": True,
        },
        "groups": {
            "state": state_groups,
            "signal": signal_groups,
            "satellite": satellite_groups,
        },
        "segment_contract": {
            "segment_count": max((item["segment"] for item in assignments), default=-1) + 1,
            "boundary_events": len(boundary_events),
        },
        "gate_observations": gate_observations,
        "events": {"count": len(boundary_events) + len(residual_events)},
        "_transitions": transitions,
        "_events": boundary_events + residual_events,
    }
    return report, boundary_events + residual_events


def _source_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase57 source {name}", pin["sha256"])
        try:
            contents[name] = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise _fail(f"Phase57 source is not UTF-8: {name}") from exc
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    adapter_header = contents["android_raw_gnss_header"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    config = contents["fgo_config_header"]
    cli = contents["gnss_fgo_cli"]
    doppler = contents["doppler_contract"]
    upstream = contents["observable_upstream_preprocessing"]
    adapter_lower = adapter.lower()
    observation_lower = observation.lower()
    fgo_lower = fgo.lower()
    config_lower = config.lower()
    return {
        "source_hashes": hashes,
        "adapter": {
            "parses_pseudorange_rate_mps": "pseudorangeratemeterspersecond" in adapter_lower and "pseudorange_rate_mps" in adapter_lower,
            "parses_pseudorange_rate_uncertainty_mps": "pseudorangerateuncertainty" in adapter_lower,
            "retains_candidate_uncertainty_in_observation": "pseudorange_rate_uncertainty" in adapter_lower,
            "existing_masks_present": all(token.lower() in adapter_lower for token in ("multipath_indicator", "cn0_dbhz", "doppler_masked")),
        },
        "observation": {
            "retains_pseudorange_rate_mps": "pseudorange_rate_mps" in observation_lower,
            "retains_candidate_uncertainty_mps": "pseudorange_rate_uncertainty" in observation_lower,
        },
        "fgo": {
            "consumes_candidate_uncertainty": "pseudorange_rate_uncertainty" in fgo_lower,
            "fixed_undifferenced_sigma_path": "undifferenced_doppler_sigma_mps" in fgo_lower,
            "fixed_single_difference_sigma_path": "single_difference_doppler_sigma_mps" in fgo_lower,
            "snr_derived_sigma_path": "snrpercentilesigma" in fgo_lower and "upstream_doppler_sigma" in fgo_lower,
            "upstream_quality_default_false": "use_upstream_observable_quality = false" in config_lower,
        },
        "config": {
            "fixed_undifferenced_sigma_mps_0_2": "undifferenced_doppler_sigma_mps = 0.2" in config_lower,
            "fixed_single_difference_sigma_mps_0_2": "single_difference_doppler_sigma_mps = 0.2" in config_lower,
        },
        "cli": {
            "fixed_doppler_sigma_option": "undifferenced-doppler-sigma" in cli,
            "candidate_uncertainty_option_absent": "pseudorangerateuncertainty" not in cli.lower(),
        },
        "doppler_contract": {
            "android_rate_to_rinex_negative_sign": "androidRateToRinexDoppler" in doppler and "-pseudorange_rate_mps" in doppler,
        },
        "upstream_contract": {
            "linear_percentile_present": "linearPercentile" in upstream,
            "snr_scale_present": "snrScale" in upstream,
            "doppler_sigma_divide_12": "scale / 12.0" in upstream,
        },
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase57 freeze", FREEZE_SHA256 or None)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase57 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase57-raw-read":
        raise _fail("Phase57 freeze schema/status mismatch")
    if freeze.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase57 route order changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase57 exact raw input map changed")
    for route in ROUTES:
        item = inputs[route]
        if len(str(item.get("sha256", ""))) != 64 or int(item.get("file_size", 0)) <= 0:
            raise _fail(f"Phase57 raw hash/size missing for {route}")
    source_contracts = freeze.get("authority_pins", {}).get("source_contracts", {})
    if not isinstance(source_contracts, dict) or len(source_contracts) < 8:
        raise _fail("Phase57 source contract pins incomplete")
    policy = freeze.get("input_policy", {})
    expected_policy = {
        "single_process": True,
        "raw_device_gnss_read_count_per_route": 1,
        "truth_read_count": 0,
        "phase45_payload_reads": 0,
        "phase56_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "correction_fit_or_application": False,
        "validation_holdout_access": False,
        "archive_reopens": 0,
        "rematerialization_count": 0,
        "kaggle_or_token_access": False,
        "mat_reads_or_generated": False,
        "device_wls_or_precomputed_coordinates": False,
        "SvPosition_or_SvElevation": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase57 read policy changed")
    gates = freeze.get("numeric_gates", {})
    required = {
        ("residual_coverage", "min_transitions_per_route"): 1000,
        ("residual_coverage", "min_satellites_per_route"): 3,
        ("residual_coverage", "min_finite_positive_uncertainty_fraction"): 0.5,
        ("residual_coverage", "min_pairs_per_populated_bin"): 50,
        ("uncertainty_population", "min_populated_fixed_bins_per_route"): 3,
        ("routewise_relation", "spearman_min_each_route"): 0.35,
        ("routewise_relation", "high_vs_low_p95_excess_min_m"): 0.05,
        ("routewise_relation", "high_vs_low_p95_ratio_min"): 1.5,
        ("calibration", "normalized_pair_sigma_median_range"): [0.25, 4.0],
        ("calibration", "normalized_abs_residual_le_1_fraction_range"): [0.05, 0.95],
        ("non_common_mode", "centered_abs_p95_min_m"): 0.01,
        ("non_common_mode", "centered_to_raw_median_ratio_min"): 0.1,
        ("current_fgo_adoption", "min_fixed_lower_bound_affected_fraction_each_route"): 0.1,
        ("current_fgo_adoption", "min_snr_enabled_affected_fraction_each_route"): 0.1,
        ("displacement_potential_proxy", "min_p95_m_each_route"): 0.05,
    }
    for (section, key), value in required.items():
        if gates.get(section, {}).get(key) != value:
            raise _fail(f"Phase57 numeric gate changed: {section}.{key}")
    if gates.get("routewise_relation", {}).get("allow_exact_equality") is not True:
        raise _fail("Phase57 routewise equality policy changed")
    if gates.get("composition_independence", {}).get("min_signal_families_per_route") != 2:
        raise _fail("Phase57 composition gate changed")
    if gates.get("presentation_integrity", {}).get("aggregate_recomputed_exact") is not True:
        raise _fail("Phase57 presentation gate changed")
    assertions = freeze.get("pre_read_assertions", {})
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase57 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase57 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase57 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase57-raw-read":
        raise _fail("Phase57 evaluator manifest schema/status mismatch")
    if manifest.get("freeze", {}).get("path") != _relative(FREEZE) or manifest.get("freeze", {}).get("sha256") != FREEZE_SHA256:
        raise _fail("Phase57 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    expected = {
        "operation": "raw-device-gnss-rate-uncertainty-audit-only",
        "native_solver_invoked": False,
        "single_process": True,
        "raw_reads_before_manifest": 0,
        "truth_reads": 0,
    }
    if any(evaluator.get(key) != value for key, value in expected.items()):
        raise _fail("Phase57 manifest permits forbidden execution")
    cohort = manifest.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("raw_device_gnss_reads_per_route") != 1 or cohort.get("truth_reads_per_route") != 0:
        raise _fail("Phase57 manifest route/read policy mismatch")
    forbidden = manifest.get("forbidden", [])
    required_forbidden = (
        "ground_truth.csv", "Phase45 truth-derived payload", "Phase56 metric payload",
        "navigation", "solver rerun", "trajectory rerun", "coordinates", "IMU",
        "MAT", "validation", "holdout", "archive reopen", "Kaggle/token",
    )
    if not isinstance(forbidden, list) or not all(item in forbidden for item in required_forbidden):
        raise _fail("Phase57 forbidden policy incomplete")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase57 {name} pin missing")
        if _sha256_bytes(pin_path.read_bytes()) != pin["sha256"]:
            raise _fail(f"Phase57 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], loo: dict[str, Any]) -> dict[str, Any]:
    route_checks: dict[str, dict[str, bool]] = {}
    for route, report in reports.items():
        total = int(report["rows"]["transition_count"])
        fixed_sum = sum(int(report["rate_uncertainty"]["relation"]["fixed_bins"][name]["count"]) for name in ("missing",) + FIXED_BINS)
        state_sum = sum(int(value["count"]) for value in report["groups"]["state"].values())
        signal_sum = sum(int(value["count"]) for value in report["groups"]["signal"].values())
        satellite_sum = sum(int(value["count"]) for value in report["groups"]["satellite"].values())
        route_checks[route] = {
            "fixed_bin_counts_sum": fixed_sum == total,
            "state_group_counts_sum": state_sum == total,
            "signal_group_counts_sum": signal_sum == total,
            "satellite_group_counts_sum": satellite_sum == total,
            "raw_hash_bytes_exact": int(report["input"]["bytes"]) > 0 and len(str(report["input"]["sha256"])) == 64,
        }
    medians = aggregate.get("route_median_abs_centered_residual_m", {})
    retained = tuple(medians) == ROUTES and len(medians) == 4
    values = list(medians.values())
    aggregate_exact = (
        aggregate.get("route_median_abs_centered_residual_aggregate_m") == _median(values)
        and aggregate.get("route_median_abs_centered_residual_mad_m") == _mad(values)
    )
    return {
        "route_checks": route_checks,
        "all_group_counts_sum": all(all(checks.values()) for checks in route_checks.values()),
        "four_route_medians_retained": retained,
        "aggregate_recomputed_exact": aggregate_exact,
        "loo_fold_count_exact": len(loo.get("folds", [])) == 4,
    }


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    for omitted in ROUTES:
        items = [item for route, report in reports.items() if route != omitted for item in report["_transitions"]]
        relation = _relation_summary(items)
        high_low = relation["high_low"]
        excess = high_low["p95_excess_m"]
        folds.append({
            "omitted_route": omitted,
            "transition_count": len(items),
            "spearman": relation["spearman_pair_uncertainty_abs_centered_residual"],
            "high_low_excess_m": excess,
            "high_low_ratio": high_low["p95_ratio"],
            "direction_nonnegative": relation["spearman_pair_uncertainty_abs_centered_residual"] >= 0.0 and (excess is None or excess >= 0.0),
        })
    return {
        "folds": folds,
        "direction_stable": bool(folds) and all(bool(item["direction_nonnegative"]) for item in folds),
    }


def _strip_internal(value: Any) -> Any:
    if isinstance(value, dict):
        return {key: _strip_internal(item) for key, item in value.items() if not str(key).startswith("_")}
    if isinstance(value, list):
        return [_strip_internal(item) for item in value]
    return value


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase57 output already exists: {output_root}")
    phase56_policy, _ = _load_json_once(
        ROOT / freeze["authority_pins"]["phase56_policy_record"]["path"],
        "Phase57 Phase56 policy record",
        freeze["authority_pins"]["phase56_policy_record"]["sha256"],
    )
    if phase56_policy.get("status") != "phase56-no-go-bias-uncertainty-duplicate-common-mode":
        raise _fail("Phase56 policy status changed")
    source = _source_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    reads: dict[str, Any] = {
        "single_process": True,
        "raw_device_gnss_reads": {},
        "raw_device_gnss_read_count_total": 0,
        "truth_reads": 0,
        "phase45_payload_reads": 0,
        "phase56_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "correction_implementations": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "validation_holdout_reads": 0,
        "kaggle_token_access": 0,
        "mat_reads_or_generated": 0,
        "device_wls_or_precomputed_coordinates": 0,
        "SvPosition_or_SvElevation": 0,
        "source_static_reads": len(source["source_hashes"]),
        "phase56_policy_record_reads": 1,
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase57 raw device_gnss {route}", pin["sha256"], int(pin["file_size"]))
        epochs, metadata = phase48._parse_payload(payload)
        del payload
        report, events = _route_report(route, epochs, metadata, path, digest)
        reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
    route_medians = {route: float(report["raw_code_rate_residual"]["clock_centered_abs_distribution_m"]["median"]) for route, report in reports.items()}
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_centered_residual_m": route_medians,
        "route_median_abs_centered_residual_aggregate_m": _median(route_medians.values()),
        "route_median_abs_centered_residual_mad_m": _mad(route_medians.values()),
        "pairwise_route_median_distances_m": [
            {"route_a": left, "route_b": right, "distance_m": abs(route_medians[left] - route_medians[right])}
            for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]
        ],
    }
    loo = _loo(reports)
    integrity = _presentation_integrity(reports, aggregate, loo)
    min_transitions = min((report["gate_observations"]["transition_count"] for report in reports.values()), default=0)
    min_sats = min((report["gate_observations"]["satellite_count"] for report in reports.values()), default=0)
    min_uncertainty_fraction = min((report["gate_observations"]["finite_positive_uncertainty_fraction"] for report in reports.values()), default=0.0)
    min_bins = min((report["gate_observations"]["populated_fixed_bins"] for report in reports.values()), default=0)
    spearman = [float(report["gate_observations"]["spearman"]) for report in reports.values()]
    high_excess = [report["gate_observations"]["high_low_excess_m"] for report in reports.values()]
    high_ratio = [report["gate_observations"]["high_low_ratio"] for report in reports.values()]
    calibration_medians = [float(report["gate_observations"]["normalized_median"]) for report in reports.values()]
    calibration_fractions = [float(report["gate_observations"]["normalized_le_one_fraction"]) for report in reports.values()]
    noncommon_p95 = [float(report["gate_observations"]["noncommon_centered_p95_m"]) for report in reports.values()]
    noncommon_ratios = [float(report["gate_observations"]["centered_to_raw_median_ratio"]) for report in reports.values()]
    fixed_affected = [float(report["gate_observations"]["fixed_lower_bound_affected_fraction"]) for report in reports.values()]
    snr_affected = [float(report["gate_observations"]["snr_enabled_affected_fraction"]) for report in reports.values()]
    displacement_p95 = [float(report["gate_observations"]["displacement_potential_proxy_p95_m"]) for report in reports.values()]
    composition = [sum(1 for group in report["groups"]["signal"].values() if int(group["count"]) > 0) for report in reports.values()]
    finite_core = all(
        bool(report["gate_observations"]["all_finite_residuals"])
        and all(_is_finite(value) for value in report["gate_observations"].values() if isinstance(value, (int, float)))
        for report in reports.values()
    )
    raw_integrity = all(
        int(report["rows"]["repeated_epoch_key_count"]) == 0
        and int(report["rows"]["nonmonotonic_epoch_key_count"]) == 0
        and bool(report["headers"]["candidate_field"])
        for report in reports.values()
    )
    route_count_pass = len(reports) == 4 and tuple(reports) == ROUTES
    coverage_pass = min_transitions >= 1000 and min_sats >= 3 and min_uncertainty_fraction >= 0.50
    population_pass = min_bins >= 3 and all(
        sum(int(report["rate_uncertainty"]["relation"]["fixed_bins"][name]["count"]) for name in FIXED_BINS if report["rate_uncertainty"]["relation"]["fixed_bins"][name]["count"] > 0) >= 50
        for report in reports.values()
    )
    relation_pass = (
        all(value >= 0.35 for value in spearman)
        and all(value is not None and float(value) >= 0.05 for value in high_excess)
        and all(value is not None and float(value) >= 1.5 for value in high_ratio)
        and all(bool(report["rate_uncertainty"]["relation"]["fixed_bin_median_non_decreasing"]) and bool(report["rate_uncertainty"]["relation"]["fixed_bin_p95_non_decreasing"]) for report in reports.values())
    )
    calibration_pass = all(0.25 <= value <= 4.0 for value in calibration_medians) and all(0.05 <= value <= 0.95 for value in calibration_fractions)
    noncommon_pass = all(value >= 0.01 for value in noncommon_p95) and all(value >= 0.10 for value in noncommon_ratios)
    adoption_pass = all(value >= 0.10 for value in fixed_affected) and all(value >= 0.10 for value in snr_affected)
    displacement_pass = all(value >= 0.05 for value in displacement_p95)
    composition_pass = all(value >= 2 for value in composition)
    source_surface_pass = (
        source["adapter"]["parses_pseudorange_rate_mps"]
        and not source["adapter"]["parses_pseudorange_rate_uncertainty_mps"]
        and source["fgo"]["fixed_undifferenced_sigma_path"]
        and source["fgo"]["fixed_single_difference_sigma_path"]
        and source["fgo"]["snr_derived_sigma_path"]
        and source["fgo"]["upstream_quality_default_false"]
        and source["cli"]["candidate_uncertainty_option_absent"]
    )
    gate_rows = [
        {"name": "route_count", "observed": len(reports), "required": 4, "passed": route_count_pass},
        {"name": "raw_input_integrity", "observed": {"hash_bytes": True, "finite_core": finite_core, "nonmonotonic_zero": raw_integrity, "candidate_field_header_all_routes": raw_integrity}, "passed": raw_integrity and finite_core},
        {"name": "residual_coverage", "observed_min_transitions": min_transitions, "observed_min_satellites": min_sats, "observed_min_uncertainty_fraction": min_uncertainty_fraction, "passed": coverage_pass},
        {"name": "uncertainty_population", "observed_min_fixed_bins": min_bins, "required_fixed_bins": 3, "passed": population_pass},
        {"name": "routewise_relation", "observed_spearman": spearman, "observed_high_low_excess_m": high_excess, "observed_high_low_ratio": high_ratio, "passed": relation_pass},
        {"name": "calibration", "observed_normalized_median": calibration_medians, "observed_le_one_fraction": calibration_fractions, "passed": calibration_pass},
        {"name": "non_common_mode", "observed_centered_p95_m": noncommon_p95, "observed_centered_to_raw_median_ratio": noncommon_ratios, "passed": noncommon_pass},
        {"name": "current_fgo_sigma_impact_proxy", "observed_fixed_affected_fraction": fixed_affected, "observed_snr_enabled_affected_fraction": snr_affected, "source_surface_contract": source_surface_pass, "passed": adoption_pass},
        {"name": "displacement_potential_proxy", "observed_p95_m": displacement_p95, "passed": displacement_pass},
        {"name": "loo_mapping_direction", "observed": loo, "passed": bool(loo["direction_stable"])},
        {"name": "composition_independence", "observed_signal_family_counts": composition, "passed": composition_pass},
        {"name": "presentation_integrity", "observed": integrity, "passed": bool(integrity["all_group_counts_sum"] and integrity["four_route_medians_retained"] and integrity["aggregate_recomputed_exact"] and integrity["loo_fold_count_exact"])},
    ]
    passed = all(bool(row["passed"]) for row in gate_rows)
    if passed:
        status = "go-rate-uncertainty-sigma-floor-concept-authorized"
        strongest = "The source-exact rate uncertainty passes all frozen raw-only coverage, relation, calibration, non-common-mode, FGO impact-proxy, LOO, composition, and integrity gates."
        next_factor = None
    elif not raw_integrity or not finite_core or not integrity["all_group_counts_sum"] or not integrity["four_route_medians_retained"] or not integrity["aggregate_recomputed_exact"]:
        status = "no-go-evaluator-integrity-failure"
        strongest = "The one-shot raw audit failed a frozen input or presentation-integrity invariant; no C++ change is authorized."
        next_factor = "raw Android per-satellite Cn0DbHz/Doppler residual calibration"
    elif not coverage_pass or not population_pass:
        status = "no-go-rate-uncertainty-insufficient-raw-coverage"
        strongest = "The eligible raw code/rate transition or source uncertainty-bin population is insufficient for four-route identification under the frozen minimums."
        next_factor = "raw Android per-satellite Cn0DbHz/Doppler residual calibration"
    elif not relation_pass or not calibration_pass or not noncommon_pass:
        status = "no-go-rate-uncertainty-not-stable-or-material"
        strongest = "PseudorangeRateUncertaintyMetersPerSecond does not show the frozen routewise monotonic, materially separated, calibrated, non-common-mode relation to the centered code/rate residual."
        next_factor = "raw Android per-satellite Cn0DbHz/Doppler residual calibration"
    elif not adoption_pass or not displacement_pass:
        status = "no-go-rate-uncertainty-insufficient-fgo-impact"
        strongest = "The raw field does not affect a sufficient fraction of the actual configured Doppler-sigma proxy or lacks the frozen one-second displacement-potential proxy; no native floor is authorized."
        next_factor = "raw Android per-satellite Cn0DbHz/Doppler residual calibration"
    else:
        status = "no-go-rate-uncertainty-composition-or-loo-failure"
        strongest = "The rate-uncertainty relation is not stable under the frozen leave-one-route-out or within-signal composition gates."
        next_factor = "raw Android per-satellite Cn0DbHz/Doppler residual calibration"
    reads["raw_device_gnss_read_count_total"] = len(reports)
    result = {
        "schema_version": SCHEMA,
        "phase": 57,
        "execution_label": "Luna Max",
        "status": status,
        "mode": "raw-device-gnss-pseudorange-rate-uncertainty-audit-only",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "phase56_policy": {"status": phase56_policy.get("status"), "next_factor": phase56_policy.get("decision", {}).get("next_single_nonduplicate_factor")},
        "read_accounting": reads,
        "routes": {route: _strip_internal(report) for route, report in reports.items()},
        "aggregate": aggregate,
        "loo": loo,
        "source_contract": source,
        "presentation_integrity": integrity,
        "gates": {"all_passed": passed, "rows": gate_rows, "fixed_thresholds": freeze["numeric_gates"]},
        "decision": {
            "audit_only": True,
            "native_correction_authorized": False,
            "implementation_stage_authorized": passed,
            "candidate_formula_if_authorized": "sigma_mps=max(existing_doppler_sigma_mps, raw_rate_uncertainty_mps), coefficient 1, no cap, FGO only, no SPP",
            "strongest_finding": strongest,
            "next_single_raw_physical_factor": next_factor,
            "truth_or_metric_payload_used": False,
            "zero_point_782": "not evaluated without truth",
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
        },
        "events": {"count": len(all_events), "table": "phase57_pixel5_rate_uncertainty.events.json"},
    }
    output_root.mkdir(parents=True, exist_ok=False)
    routes_path = output_root / "phase57_pixel5_rate_uncertainty.routes.json"
    events_path = output_root / "phase57_pixel5_rate_uncertainty.events.json"
    result_path = output_root / "phase57_pixel5_rate_uncertainty.json"
    manifest_path = output_root / "phase57_pixel5_rate_uncertainty.manifest.json"
    routes_payload = {"schema_version": SCHEMA + ".routes", "routes": result["routes"]}
    events_payload = {"schema_version": SCHEMA + ".events", "events": _strip_internal(all_events)}
    routes_bytes = _atomic_json(routes_path, routes_payload)
    events_bytes = _atomic_json(events_path, events_payload)
    result["output_artifacts"] = {
        _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
        _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
    }
    result_bytes = _atomic_json(result_path, result)
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 57,
        "freeze_sha256": FREEZE_SHA256,
        "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "read_accounting": reads,
        "artifacts": {
            _relative(result_path): {"bytes": len(result_bytes), "sha256": _sha256_bytes(result_bytes)},
            _relative(routes_path): {"bytes": len(routes_bytes), "sha256": _sha256_bytes(routes_bytes)},
            _relative(events_path): {"bytes": len(events_bytes), "sha256": _sha256_bytes(events_bytes)},
        },
    }
    _atomic_json(manifest_path, output_manifest)
    return result


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify freeze and evaluator manifest without raw/truth reads")
    parser.add_argument("--audit", action="store_true", help="run the one-shot truth-free raw audit")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.verify_freeze == args.audit:
        parser.error("choose exactly one of --verify-freeze or --audit")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase57 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({"status": result["status"], "raw_reads": result["read_accounting"]["raw_device_gnss_read_count_total"], "next_factor": result["decision"]["next_single_raw_physical_factor"]}, sort_keys=True))
        return 0 if result["gates"]["all_passed"] else 3
    except Phase57Error as exc:
        print(f"phase57 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
