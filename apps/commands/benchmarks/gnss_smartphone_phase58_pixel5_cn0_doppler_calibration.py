#!/usr/bin/env python3
"""Truth-free Phase58 C/N0/Doppler residual-calibration audit.

The audit reads the four frozen Pixel5 Android ``device_gnss.csv`` files once
in one process.  It reuses only the reviewed Phase25 raw pseudorange parser,
then relates Cn0DbHz to the same-satellite code/range-rate closure residual.
The source-supported shape is the 20-dB exponential already used by the
upstream SNR path.  A robust scale is fitted only from centered raw closure
residuals in route-LOO folds; no truth, navigation, solver, coordinates, IMU,
or prior metric payload is opened.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase58_pixel5_cn0_doppler_calibration_freeze_v1.json"
FREEZE_SHA256 = "8bd52f165d69279b4eb92dac82e9b470d4656de639ccdf327ae0613df7137448"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase58_pixel5_cn0_doppler_calibration_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase58-pixel5-cn0-doppler-calibration-v1"

SCHEMA = "smartphone-r5-phase58-pixel5-cn0-doppler-calibration.v1"
FREEZE_SCHEMA = "smartphone-r5-phase58-pixel5-cn0-doppler-calibration-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase58-pixel5-cn0-doppler-calibration-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = 299_792_458.0
TRANSITION_MAX_NS = 1_500_000_000
MIN_P = 10_000_000.0
MAX_P = 40_000_000.0
FIXED_DOPPLER_SIGMA_MPS = 0.2
CN0_REFERENCE_DBHZ = 40.0
CN0_BIN_LABELS = ("20_to_25", "25_to_30", "30_to_35", "35_to_40", ">=40")
CN0_BIN_EDGES = (20.0, 25.0, 30.0, 35.0, 40.0)
EVENT_RESIDUAL_THRESHOLD_M = 2.0


class Phase58Error(ValueError):
    """Raised when the immutable Phase58 audit contract is violated."""


def _fail(message: str) -> Phase58Error:
    return Phase58Error(message)


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
        raise _fail(f"forbidden Phase58 path: {path}")


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
) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
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
    descriptor, temporary = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent)
    )
    try:
        with open(descriptor, "wb", closefd=True) as handle:
            handle.write(payload)
            handle.flush()
            import os
            os.fsync(handle.fileno())
        os.replace(temporary, path)
        temporary = ""
    finally:
        if temporary:
            try:
                import os
                os.unlink(temporary)
            except FileNotFoundError:
                pass


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes(value)
    _atomic_write(path, payload)
    return payload


def _finite(value: Any) -> bool:
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


def _shape(cn0_dbhz: float, reference: float = CN0_REFERENCE_DBHZ) -> float:
    """Source-supported 20-dB monotonic C/N0 scale, dimensionless."""
    if not _finite(cn0_dbhz) or not _finite(reference):
        return math.nan
    value = 10.0 ** (-(float(cn0_dbhz) - float(reference)) / 20.0)
    return value if _finite(value) and value > 0.0 else math.nan


def _candidate_sigma(cn0_dbhz: float | None, alpha_mps: float) -> float | None:
    if cn0_dbhz is None or not _finite(cn0_dbhz) or not _finite(alpha_mps) or alpha_mps <= 0.0:
        return None
    model = alpha_mps * _shape(float(cn0_dbhz))
    if not _finite(model) or model <= 0.0:
        return None
    return max(FIXED_DOPPLER_SIGMA_MPS, model)


def _linear_percentile(values: Iterable[float], percentile: float) -> float:
    """The existing source's MATLAB ``linearPercentile`` convention."""
    data = sorted(float(value) for value in values if _finite(value))
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


def _cn0_bin(value: float | None) -> str:
    if value is None or not _finite(value) or value < CN0_BIN_EDGES[0]:
        return "missing_or_below_20"
    for index in range(len(CN0_BIN_LABELS) - 1):
        if value < CN0_BIN_EDGES[index + 1]:
            return CN0_BIN_LABELS[index]
    return CN0_BIN_LABELS[-1]


def _signal_family(system: str, signal: str) -> str:
    return f"{system}:{signal}"


def _source_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    pins = freeze["authority"]["source_contracts"]
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in pins.items():
        payload, digest = _read_bytes_once(
            ROOT / pin["path"], f"Phase58 static source {name}", pin["sha256"]
        )
        contents[name] = payload.decode("utf-8")
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    config = contents["fgo_config_header"]
    cli = contents["gnss_fgo_cli"]
    upstream = contents["observable_upstream_preprocessing"]
    adapter_lower = adapter.lower()
    fgo_lower = fgo.lower()
    upstream_lower = upstream.lower()
    config_lower = config.lower()
    cli_lower = cli.lower()
    return {
        "source_hashes": hashes,
        "adapter": {
            "parses_cn0_dbhz": "cn0dbhz" in adapter_lower,
            "parses_pseudorange_rate": "pseudorangeratemeterspersecond" in adapter_lower,
            "existing_low_cn0_mask": "cn0dbhz" in adapter_lower and "20" in adapter_lower,
        },
        "observation": {
            "retains_cn0_as_snr": "snr" in observation.lower(),
            "candidate_calibration_field_absent": "cn0dbhz" not in observation.lower(),
        },
        "fgo": {
            "uses_cn0_snr": "snrpercentilesigma" in fgo_lower and "upstream_doppler_sigma" in fgo_lower,
            "uses_source_doppler_divide_12": "upstream_doppler_sigma" in fgo_lower and "snrpercentilesigma" in fgo_lower,
            "upstream_quality_switch": "use_upstream_observable_quality" in fgo_lower,
            "fixed_undifferenced_sigma_path": "undifferenced_doppler_sigma_mps" in fgo_lower,
            "fixed_single_difference_sigma_path": "single_difference_doppler_sigma_mps" in fgo_lower,
            "candidate_cn0_calibration_absent": "cn0_calibration" not in fgo_lower,
        },
        "config": {
            "upstream_quality_default_false": "use_upstream_observable_quality = false" in config_lower,
            "fixed_sigma_default_0_2": "undifferenced_doppler_sigma_mps = 0.2" in config_lower,
        },
        "cli": {
            "upstream_quality_option_present": "upstream-observable-quality" in cli_lower,
            "candidate_cn0_option_absent": "cn0" not in cli_lower or "calibration" not in cli_lower,
        },
        "upstream_contract": {
            "twenty_db_exponent": "pow(10.0, -(snr_dbhz - percentile85_dbhz) / 20.0)" in upstream,
            "matlab_linear_percentile": "linearpercentile" in upstream_lower,
            "doppler_sigma_divide_12": "case 'd': return scale / 12.0" in upstream_lower,
        },
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase58 freeze", FREEZE_SHA256)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase58 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase58-raw-read":
        raise _fail("Phase58 freeze schema/status mismatch")
    cohort = freeze.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("route_count") != 4 or cohort.get("route_order_fixed") is not True:
        raise _fail("Phase58 route contract changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase58 exact raw input map changed")
    for route in ROUTES:
        pin = inputs[route]
        if len(str(pin.get("sha256", ""))) != 64 or int(pin.get("file_size", 0)) <= 0:
            raise _fail(f"Phase58 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected_policy = {
        "single_process": True,
        "raw_device_gnss_read_count_per_route": 1,
        "raw_device_imu_reads": 0,
        "truth_reads": 0,
        "navigation_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_access": False,
        "archive_reopens": 0,
        "rematerialization_count": 0,
        "kaggle_or_token_access": False,
        "device_wls_or_precomputed_coordinates": False,
        "SvPosition_or_SvElevation": False,
        "phase45_payload_reads": 0,
        "phase57_metric_payload_reads": 0,
        "correction_fit_or_application": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase58 read policy changed")
    gates = freeze.get("numeric_gates", {})
    checks = (
        ("coverage", "min_transitions_per_route", 1000),
        ("coverage", "min_satellites_per_route", 3),
        ("coverage", "min_finite_positive_cn0_fraction", 0.5),
        ("coverage", "min_finite_pair_cn0_fraction", 0.5),
        ("coverage", "min_pairs_per_cn0_bin", 50),
        ("coverage", "min_populated_cn0_bins_per_route", 4),
        ("routewise_monotonicity", "spearman_cn0_vs_abs_centered_rate_residual_max", -0.25),
        ("loo_calibration", "folds", 4),
        ("loo_calibration", "fit_routes_per_fold", 3),
        ("loo_calibration", "alpha_max_over_min_ratio", 3.0),
        ("loo_calibration", "alpha_coefficient_of_variation_max", 0.5),
        ("non_common_mode", "per_satellite_min_groups_each_route", 3),
        ("non_common_mode", "per_satellite_min_pairs", 50),
        ("non_common_mode", "within_satellite_spearman_max", -0.15),
        ("non_common_mode", "within_signal_spearman_max", -0.15),
        ("current_factor_impact", "candidate_above_configured_sigma_fraction_min", 0.1),
        ("current_factor_impact", "candidate_p95_sigma_excess_mps_min", 0.05),
    )
    for section, key, expected in checks:
        if gates.get(section, {}).get(key) != expected:
            raise _fail(f"Phase58 numeric gate changed: {section}.{key}")
    if gates.get("cn0_bins", {}).get("fixed_edges_dbhz") != list(CN0_BIN_EDGES):
        raise _fail("Phase58 C/N0 bin edges changed")
    if gates.get("cn0_bins", {}).get("labels") != list(CN0_BIN_LABELS):
        raise _fail("Phase58 C/N0 bin labels changed")
    if gates.get("routewise_monotonicity", {}).get("allow_exact_equality") is not True:
        raise _fail("Phase58 monotonicity equality policy changed")
    if gates.get("loo_calibration", {}).get("direction_stable") is not True:
        raise _fail("Phase58 LOO direction policy changed")
    if gates.get("presentation_integrity", {}).get("aggregate_recomputed_exact") is not True:
        raise _fail("Phase58 presentation integrity gate changed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(value is not False for value in pre.values()):
        raise _fail("Phase58 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase58 evaluator manifest")
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase58 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase58-raw-read":
        raise _fail("Phase58 manifest schema/status mismatch")
    freeze_pin = manifest.get("freeze", {})
    if freeze_pin.get("path") != _relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase58 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("single_process") is not True or evaluator.get("native_solver_invoked") is not False or evaluator.get("raw_reads_per_route") != 1:
        raise _fail("Phase58 manifest permits forbidden execution")
    if manifest.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase58 manifest route order mismatch")
    forbidden = manifest.get("forbidden", [])
    required = ("ground_truth.csv", "navigation", "solver/trajectory", "coordinates", "MAT", "Phase57 metric payload", "native correction")
    if not isinstance(forbidden, list) or not all(item in forbidden for item in required):
        raise _fail("Phase58 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase58 {name} pin missing")
        actual = _sha256_bytes(pin_path.read_bytes())
        if actual != pin["sha256"]:
            raise _fail(f"Phase58 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _prepare_parser() -> Any:
    if str(ROOT / "apps/commands/benchmarks") not in sys.path:
        sys.path.insert(0, str(ROOT / "apps/commands/benchmarks"))
    import gnss_smartphone_phase48_pixel5_raw_code_rate_uncertainty_audit as parser  # noqa: E402
    return parser


def _transition_items(epochs: Sequence[Any], parser: Any) -> tuple[list[dict[str, Any]], list[dict[str, Any]]]:
    by_key: dict[tuple[str, int, str], list[Any]] = defaultdict(list)
    for epoch in epochs:
        for row in epoch.rows:
            by_key[row.sat_signal].append(row)
    transitions: list[dict[str, Any]] = []
    events: list[dict[str, Any]] = []
    for _, rows in sorted(by_key.items()):
        rows.sort(key=lambda row: (row.utc_ms, row.row_number))
        for previous, current in zip(rows, rows[1:]):
            dt_ns = current.time_ns - previous.time_ns
            if dt_ns <= 0 or dt_ns > TRANSITION_MAX_NS:
                continue
            if current.hcdc != previous.hcdc or current.segment != previous.segment:
                continue
            supported_system = {"GPS", "GLONASS", "GALILEO", "BEIDOU"}
            supported_signals = set(parser.SIGNAL_MAP.values())
            if (previous.system not in supported_system or current.system not in supported_system or
                    previous.signal not in supported_signals or current.signal not in supported_signals):
                continue
            if previous.rate_mps is None or current.rate_mps is None:
                continue
            if previous.pseudorange_m is None or current.pseudorange_m is None:
                continue
            if previous.code_masked or current.code_masked:
                continue
            dt = dt_ns / 1e9
            raw = (float(current.pseudorange_m) - float(previous.pseudorange_m)) - 0.5 * (
                float(previous.rate_mps) + float(current.rate_mps)
            ) * dt
            if not _finite(raw):
                continue
            previous_cn0 = float(previous.cn0) if previous.cn0 is not None and _finite(previous.cn0) and float(previous.cn0) > 0.0 else None
            current_cn0 = float(current.cn0) if current.cn0 is not None and _finite(current.cn0) and float(current.cn0) > 0.0 else None
            pair_cn0 = (previous_cn0 + current_cn0) / 2.0 if previous_cn0 is not None and current_cn0 is not None else None
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
                "previous_cn0_dbhz": previous_cn0,
                "cn0_dbhz": current_cn0,
                "pair_cn0_dbhz": pair_cn0,
                "raw_residual_m": float(raw),
            })
    by_clock: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in transitions:
        by_clock[(item["utcTimeMillis"], item["hcdc"])].append(item["raw_residual_m"])
    for item in transitions:
        center = _median(by_clock[(item["utcTimeMillis"], item["hcdc"])])
        item["clock_group_median_m"] = center
        item["centered_residual_m"] = item["raw_residual_m"] - center
        item["abs_centered_residual_m"] = abs(item["centered_residual_m"])
        item["abs_centered_rate_residual_mps"] = item["abs_centered_residual_m"] / item["dt_s"]
        if item["abs_centered_residual_m"] >= EVENT_RESIDUAL_THRESHOLD_M:
            events.append({
                "kind": "code_rate_residual_outlier",
                "utcTimeMillis": item["utcTimeMillis"],
                "system": item["system"],
                "svid": item["svid"],
                "signal": item["signal"],
                "abs_centered_residual_m": item["abs_centered_residual_m"],
            })
    return transitions, events


def _summary(values: Sequence[float]) -> dict[str, Any]:
    return _distribution(values, 0.0)


def _bin_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    buckets: dict[str, list[float]] = {label: [] for label in CN0_BIN_LABELS}
    for item in items:
        label = _cn0_bin(item.get("pair_cn0_dbhz"))
        if label in buckets:
            buckets[label].append(float(item["abs_centered_rate_residual_mps"]))
    result: dict[str, Any] = {}
    for label in CN0_BIN_LABELS:
        values = buckets[label]
        result[label] = {
            "count": len(values),
            "median_mps": _median(values),
            "p95_mps": _percentile(values, 0.95),
            "max_mps": max(values) if values else 0.0,
        }
    populated = [label for label in CN0_BIN_LABELS if buckets[label]]
    medians = [result[label]["median_mps"] for label in populated]
    p95s = [result[label]["p95_mps"] for label in populated]
    return {
        "ordered": result,
        "populated_labels": populated,
        "populated_count": len(populated),
        "median_non_increasing": all(left >= right for left, right in zip(medians, medians[1:])),
        "p95_non_increasing": all(left >= right for left, right in zip(p95s, p95s[1:])),
    }


def _finite_cn0_items(items: Sequence[dict[str, Any]]) -> list[dict[str, Any]]:
    return [
        item for item in items
        if item.get("pair_cn0_dbhz") is not None
        and _finite(item["pair_cn0_dbhz"])
        and item["pair_cn0_dbhz"] >= CN0_BIN_EDGES[0]
        and _finite(item.get("abs_centered_rate_residual_mps"))
    ]


def _fit_alpha(items: Sequence[dict[str, Any]]) -> float:
    ratios: list[float] = []
    for item in _finite_cn0_items(items):
        shape = _shape(float(item["pair_cn0_dbhz"]))
        rate_residual = float(item["abs_centered_rate_residual_mps"])
        if shape > 0.0 and _finite(rate_residual):
            ratios.append(rate_residual / shape)
    alpha = _median(ratios)
    return alpha if _finite(alpha) and alpha > 0.0 else math.nan


def _snr_doppler_sigma(cn0: float | None, p85: float | None) -> float | None:
    if cn0 is None or p85 is None or not _finite(cn0) or not _finite(p85):
        return None
    sigma = (10.0 ** (-(float(cn0) - float(p85)) / 20.0)) / 12.0
    return sigma if _finite(sigma) and sigma > 0.0 else None


def _route_p85(items: Sequence[dict[str, Any]]) -> float:
    values = [float(item["cn0_dbhz"]) for item in items if item.get("cn0_dbhz") is not None and _finite(item["cn0_dbhz"])]
    return _linear_percentile(values, 85.0) if values else math.nan


def _group_relation(items: Sequence[dict[str, Any]], key_fn: Any, min_pairs: int) -> dict[str, Any]:
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in items:
        if item.get("pair_cn0_dbhz") is not None:
            groups[str(key_fn(item))].append(item)
    report: dict[str, Any] = {}
    for name, group in sorted(groups.items()):
        x = [float(item["pair_cn0_dbhz"]) for item in group]
        y = [float(item["abs_centered_rate_residual_mps"]) for item in group]
        rho = _spearman(x, y) if len(group) >= 2 else 0.0
        report[name] = {
            "count": len(group),
            "spearman_cn0_vs_abs_centered_rate_residual": rho,
            "residual_mps": _summary(y),
            "cn0_dbhz": _summary(x),
            "meets_min_pairs": len(group) >= min_pairs,
        }
    return report


def _calibration(items: Sequence[dict[str, Any]], alpha: float) -> dict[str, Any]:
    normalized: list[float] = []
    sigmas: list[float] = []
    affected: list[float] = []
    snr_affected: list[float] = []
    p85 = _route_p85(items)
    for item in items:
        cn0 = item.get("cn0_dbhz")
        sigma_model = _candidate_sigma(cn0, alpha)
        if sigma_model is None:
            continue
        residual = float(item["abs_centered_rate_residual_mps"])
        normalized.append(residual / sigma_model)
        sigmas.append(sigma_model)
        if sigma_model > FIXED_DOPPLER_SIGMA_MPS:
            affected.append(sigma_model - FIXED_DOPPLER_SIGMA_MPS)
        snr_sigma = _snr_doppler_sigma(cn0, p85)
        if snr_sigma is not None and sigma_model > snr_sigma:
            snr_affected.append(sigma_model - snr_sigma)
    return {
        "alpha_mps_at_40_dbhz": alpha,
        "candidate_sigma_mps": _summary(sigmas),
        "normalized_abs_centered_rate_residual": _summary(normalized),
        "normalized_le_one_fraction": sum(value <= 1.0 for value in normalized) / len(normalized) if normalized else 0.0,
        "candidate_above_fixed_sigma_count": len(affected),
        "candidate_above_fixed_sigma_fraction": len(affected) / len(sigmas) if sigmas else 0.0,
        "candidate_sigma_excess_p95_mps": _percentile(affected, 0.95) if affected else 0.0,
        "snr_p85_dbhz": p85,
        "candidate_above_exact_snr_sigma_count": len(snr_affected),
        "candidate_above_exact_snr_sigma_fraction": len(snr_affected) / len(sigmas) if sigmas else 0.0,
    }


def _route_report(route: str, epochs: Sequence[Any], metadata: dict[str, Any], path: Path, digest: str, parser: Any) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    assignments, boundaries = parser._assign_segments(epochs)
    transitions, residual_events = _transition_items(epochs, parser)
    finite = _finite_cn0_items(transitions)
    bins = _bin_summary(transitions)
    cn0 = [float(item["pair_cn0_dbhz"]) for item in finite]
    residual = [float(item["abs_centered_rate_residual_mps"]) for item in finite]
    satellites = sorted({(item["system"], int(item["svid"])) for item in transitions})
    signals = sorted({_signal_family(item["system"], item["signal"]) for item in transitions})
    satellite_groups = _group_relation(transitions, lambda item: f"{item['system']}:{item['svid']}", 50)
    signal_groups = _group_relation(transitions, lambda item: _signal_family(item["system"], item["signal"]), 50)
    report = {
        "route": route,
        "input": {"path": _relative(path), "bytes": int(path.stat().st_size), "sha256": digest},
        "rows": {
            "raw": int(metadata["raw_rows"]),
            "non_raw": int(metadata["non_raw_rows"]),
            "unsupported_signal_rows_informational": int(metadata["unsupported_signal_rows"]),
            "epochs": len(epochs),
            "raw_row_coverage": 1.0,
            "transition_count": len(transitions),
            "finite_pair_cn0_count": len(finite),
            "finite_pair_cn0_fraction": len(finite) / len(transitions) if transitions else 0.0,
            "finite_current_cn0_fraction": sum(item.get("cn0_dbhz") is not None for item in transitions) / len(transitions) if transitions else 0.0,
            "satellite_count": len(satellites),
            "satellites": [f"{system}:{svid}" for system, svid in satellites],
            "signal_families": signals,
            "repeated_epoch_key_count": int(metadata["repeated_epoch_key_count"]),
            "nonmonotonic_epoch_key_count": int(metadata["nonmonotonic_epoch_key_count"]),
        },
        "closure_residual": {
            "raw_residual_m": _summary([float(item["raw_residual_m"]) for item in transitions]),
            "centered_abs_residual_m": _summary([float(item["abs_centered_residual_m"]) for item in transitions]),
            "centered_abs_rate_residual_mps": _summary([float(item["abs_centered_rate_residual_mps"]) for item in transitions]),
            "cn0_pair_spearman": _spearman(cn0, residual),
            "cn0_pair_count": len(finite),
        },
        "cn0_bins": bins,
        "route_relation": {
            "spearman_max_gate_value": _spearman(cn0, residual),
            "median_non_increasing": bins["median_non_increasing"],
            "p95_non_increasing": bins["p95_non_increasing"],
            "satellite_groups": satellite_groups,
            "signal_groups": signal_groups,
            "satellite_groups_with_min_pairs": sum(value["meets_min_pairs"] for value in satellite_groups.values()),
            "satellite_groups_direction_nonpositive": all(value["spearman_cn0_vs_abs_centered_rate_residual"] <= -0.15 for value in satellite_groups.values() if value["meets_min_pairs"]),
            "signal_groups_with_min_pairs": sum(value["meets_min_pairs"] for value in signal_groups.values()),
            "signal_groups_direction_nonpositive": all(value["spearman_cn0_vs_abs_centered_rate_residual"] <= -0.15 for value in signal_groups.values() if value["meets_min_pairs"]),
        },
        "segment_contract": {
            "segment_count": max((int(item["segment"]) for item in assignments), default=-1) + 1,
            "boundary_events": len(boundaries),
        },
        "events": {"count": len(boundaries) + len(residual_events)},
        "_transitions": transitions,
        "_events": boundaries + residual_events,
    }
    return report, boundaries + residual_events


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    alphas: list[float] = []
    for omitted in ROUTES:
        train = [item for route, report in reports.items() if route != omitted for item in report["_transitions"]]
        heldout = reports[omitted]["_transitions"]
        alpha = _fit_alpha(train)
        alphas.append(alpha)
        calibration = _calibration(heldout, alpha)
        finite = _finite_cn0_items(heldout)
        x = [float(item["pair_cn0_dbhz"]) for item in finite]
        y = [float(item["abs_centered_rate_residual_mps"]) for item in finite]
        rho = _spearman(x, y)
        folds.append({
            "omitted_route": omitted,
            "training_routes": [route for route in ROUTES if route != omitted],
            "training_transition_count": len(train),
            "heldout_transition_count": len(heldout),
            "alpha_mps_at_40_dbhz": alpha,
            "heldout_cn0_spearman": rho,
            "heldout_normalized_median": calibration["normalized_abs_centered_rate_residual"]["median"],
            "heldout_normalized_le_one_fraction": calibration["normalized_le_one_fraction"],
            "heldout_candidate_above_fixed_sigma_fraction": calibration["candidate_above_fixed_sigma_fraction"],
            "heldout_candidate_sigma_excess_p95_mps": calibration["candidate_sigma_excess_p95_mps"],
            "direction_negative": rho <= -0.25,
        })
    finite_alphas = [value for value in alphas if _finite(value) and value > 0.0]
    alpha_ratio = max(finite_alphas) / min(finite_alphas) if finite_alphas else math.inf
    alpha_mean = statistics.fmean(finite_alphas) if finite_alphas else math.nan
    alpha_cv = (statistics.pstdev(finite_alphas) / alpha_mean) if finite_alphas and alpha_mean > 0.0 else math.inf
    return {
        "folds": folds,
        "alpha_values_mps_at_40_dbhz": alphas,
        "alpha_max_over_min_ratio": alpha_ratio,
        "alpha_coefficient_of_variation": alpha_cv,
        "direction_stable": bool(folds) and all(fold["direction_negative"] for fold in folds),
    }


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], loo: dict[str, Any]) -> dict[str, Any]:
    group_counts: dict[str, bool] = {}
    for route, report in reports.items():
        count = int(report["rows"]["finite_pair_cn0_count"])
        bins = report["cn0_bins"]["ordered"]
        group_counts[route] = sum(int(value["count"]) for value in bins.values()) == count
    retained = tuple(aggregate["route_median_abs_rate_residual_mps"]) == ROUTES and len(aggregate["route_median_abs_rate_residual_mps"]) == 4
    values = list(aggregate["route_median_abs_rate_residual_mps"].values())
    recomputed = aggregate["aggregate_median_mps"] == _median(values) and aggregate["aggregate_mad_mps"] == _mad(values)
    return {
        "all_group_counts_sum": all(group_counts.values()),
        "group_counts_by_route": group_counts,
        "four_route_medians_retained": retained,
        "aggregate_recomputed_exact": recomputed,
        "loo_fold_count_exact": len(loo.get("folds", [])) == 4,
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
        raise _fail(f"Phase58 output already exists: {output_root}")
    parser = _prepare_parser()
    source = _source_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    reads: dict[str, Any] = {
        "single_process": True,
        "raw_device_gnss_reads": {},
        "raw_device_gnss_read_count_total": 0,
        "raw_device_imu_reads": 0,
        "truth_reads": 0,
        "navigation_reads": 0,
        "solver_reruns": 0,
        "trajectory_reruns": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "kaggle_token_access": 0,
        "device_wls_or_precomputed_coordinates": 0,
        "SvPosition_or_SvElevation": 0,
        "phase45_payload_reads": 0,
        "phase57_metric_payload_reads": 0,
        "correction_implementations": 0,
        "source_static_reads": len(source["source_hashes"]),
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase58 raw device_gnss {route}", pin["sha256"], int(pin["file_size"]))
        epochs, metadata = parser._parse_payload(payload)
        del payload
        report, events = _route_report(route, epochs, metadata, path, digest, parser)
        reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1

    medians = {route: _median(item["abs_centered_rate_residual_mps"] for item in report["_transitions"]) for route, report in reports.items()}
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_rate_residual_mps": medians,
        "aggregate_median_mps": _median(medians.values()),
        "aggregate_mad_mps": _mad(medians.values()),
        "pairwise_route_median_distances_mps": [
            {"route_a": left, "route_b": right, "distance_mps": abs(medians[left] - medians[right])}
            for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]
        ],
    }
    loo = _loo(reports)
    integrity = _presentation_integrity(reports, aggregate, loo)
    min_transitions = min((int(report["rows"]["transition_count"]) for report in reports.values()), default=0)
    min_satellites = min((int(report["rows"]["satellite_count"]) for report in reports.values()), default=0)
    min_pair_fraction = min((float(report["rows"]["finite_pair_cn0_fraction"]) for report in reports.values()), default=0.0)
    min_bins = min((int(report["cn0_bins"]["populated_count"]) for report in reports.values()), default=0)
    min_bin_count = min((min((int(item["count"]) for item in report["cn0_bins"]["ordered"].values() if int(item["count"]) > 0), default=0) for report in reports.values()), default=0)
    relation_pass = all(
        float(report["closure_residual"]["cn0_pair_spearman"]) <= -0.25
        and bool(report["cn0_bins"]["median_non_increasing"])
        and bool(report["cn0_bins"]["p95_non_increasing"])
        for report in reports.values()
    )
    satellite_pass = all(
        int(report["route_relation"]["satellite_groups_with_min_pairs"]) >= 3
        and bool(report["route_relation"]["satellite_groups_direction_nonpositive"])
        for report in reports.values()
    )
    signal_pass = all(
        int(report["route_relation"]["signal_groups_with_min_pairs"]) >= 1
        and bool(report["route_relation"]["signal_groups_direction_nonpositive"])
        for report in reports.values()
    )
    coverage_pass = min_transitions >= 1000 and min_satellites >= 3 and min_pair_fraction >= 0.5 and min_bins >= 4 and min_bin_count >= 50
    calibration_pass = (
        all(0.25 <= float(fold["heldout_normalized_median"]) <= 4.0 and 0.05 <= float(fold["heldout_normalized_le_one_fraction"]) <= 0.95 for fold in loo["folds"])
        and float(loo["alpha_max_over_min_ratio"]) <= 3.0
        and float(loo["alpha_coefficient_of_variation"]) <= 0.5
        and bool(loo["direction_stable"])
    )
    impact_reports: dict[str, dict[str, Any]] = {}
    for fold in loo["folds"]:
        route = str(fold["omitted_route"])
        alpha = float(fold["alpha_mps_at_40_dbhz"])
        impact_reports[route] = _calibration(reports[route]["_transitions"], alpha)
    impact_pass = all(
        float(item["candidate_above_fixed_sigma_fraction"]) >= 0.10
        and float(item["candidate_sigma_excess_p95_mps"]) >= 0.05
        for item in impact_reports.values()
    )
    source_pass = (
        source["adapter"]["parses_cn0_dbhz"]
        and source["adapter"]["parses_pseudorange_rate"]
        and source["observation"]["retains_cn0_as_snr"]
        and source["fgo"]["uses_cn0_snr"]
        and source["fgo"]["uses_source_doppler_divide_12"]
        and source["fgo"]["fixed_undifferenced_sigma_path"]
        and source["config"]["upstream_quality_default_false"]
        and source["config"]["fixed_sigma_default_0_2"]
        and source["fgo"]["candidate_cn0_calibration_absent"]
        and source["cli"]["candidate_cn0_option_absent"]
        and source["upstream_contract"]["twenty_db_exponent"]
    )
    raw_integrity = all(
        report["input"]["sha256"] == freeze["exact_raw_inputs"][route]["sha256"]
        and report["input"]["bytes"] == int(freeze["exact_raw_inputs"][route]["file_size"])
        and report["rows"]["repeated_epoch_key_count"] == 0
        and report["rows"]["nonmonotonic_epoch_key_count"] == 0
        for route, report in reports.items()
    )
    presentation_pass = bool(integrity["all_group_counts_sum"] and integrity["four_route_medians_retained"] and integrity["aggregate_recomputed_exact"] and integrity["loo_fold_count_exact"])
    gate_rows = {
        "route_count": len(reports) == 4,
        "raw_input_integrity": raw_integrity,
        "source_contract": source_pass,
        "coverage": coverage_pass,
        "routewise_monotonicity": relation_pass,
        "loo_calibration": calibration_pass,
        "per_satellite_non_common_mode": satellite_pass,
        "per_signal_non_common_mode": signal_pass,
        "current_factor_impact": impact_pass,
        "presentation_integrity": presentation_pass,
    }
    all_passed = all(gate_rows.values())
    decision_status = "go-cn0-doppler-calibration-audit" if all_passed else "no-go-cn0-doppler-calibration-not-identifiable"
    result = {
        "schema_version": SCHEMA,
        "phase": 58,
        "execution_label": "Luna Max",
        "status": decision_status,
        "operation": "truth-free raw Android Cn0DbHz/Doppler residual calibration audit",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256, "seal_commit": "3f44cc0"},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256, "seal_commit": manifest.get("seal_commit")},
        "evaluator": {"path": _relative(Path(__file__)), "sha256": _sha256_bytes(Path(__file__).read_bytes()), "one_shot": True, "single_process": True, "parser": "Phase25 reviewed raw parser only; no prior metric payload"},
        "source_contract": source,
        "champion_config_metadata": freeze["authority"]["phase43_champion_config"],
        "fixed_model": freeze["existing_model_audit"],
        "routes": {route: _strip_internal(report) for route, report in reports.items()},
        "aggregate": aggregate,
        "loo": loo,
        "candidate_impact_loo": impact_reports,
        "gates": {"all_passed": all_passed, "observed": gate_rows, "minimums": freeze["numeric_gates"], "coverage_observed": {"min_transitions": min_transitions, "min_satellites": min_satellites, "min_pair_cn0_fraction": min_pair_fraction, "min_populated_bins": min_bins, "min_bin_count": min_bin_count}},
        "decision": {
            "audit_only": not all_passed,
            "native_correction_authorized": False,
            "implementation_stage_authorized": all_passed,
            "strongest_finding": "The raw C/N0-to-closure relation and LOO calibration are evaluated without truth; any gate failure keeps the candidate diagnostic-only and prevents native implementation.",
            "next_single_raw_physical_factor_if_no_go": "none-authorized; preserve Phase43 champion until a separately frozen source-supported factor is selected",
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "truth_or_metric_payload_used": False,
            "zero_point_782": "not evaluated without truth",
        },
        "read_accounting": reads,
        "events": {"count": len(all_events), "table": "phase58_pixel5_cn0_doppler_calibration.events.json"},
        "presentation_integrity": integrity,
        "output_artifacts": {},
    }
    output_root.mkdir(parents=True, exist_ok=False)
    routes_path = output_root / "phase58_pixel5_cn0_doppler_calibration.routes.json"
    events_path = output_root / "phase58_pixel5_cn0_doppler_calibration.events.json"
    manifest_path = output_root / "phase58_pixel5_cn0_doppler_calibration.manifest.json"
    result_path = output_root / "phase58_pixel5_cn0_doppler_calibration.json"
    route_bytes = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "routes": result["routes"], "aggregate": aggregate, "loo": loo, "candidate_impact_loo": impact_reports})
    event_bytes = _atomic_json(events_path, {"schema_version": SCHEMA + ".events", "events": all_events})
    manifest_value = {"schema_version": SCHEMA + ".manifest", "status": decision_status, "result_sha256": "pending", "routes_sha256": _sha256_bytes(route_bytes), "events_sha256": _sha256_bytes(event_bytes), "raw_reads": reads["raw_device_gnss_read_count_total"], "truth_reads": reads["truth_reads"]}
    result["output_artifacts"] = {
        "routes": {"path": _relative(routes_path), "bytes": len(route_bytes), "sha256": _sha256_bytes(route_bytes)},
        "events": {"path": _relative(events_path), "bytes": len(event_bytes), "sha256": _sha256_bytes(event_bytes)},
    }
    result_bytes = _atomic_json(result_path, result)
    # The manifest records the final result hash after result construction;
    # this is output bookkeeping only and never touches raw input.
    manifest_value["result_sha256"] = _sha256_bytes(result_bytes)
    _atomic_json(manifest_path, manifest_value)
    return result


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true")
    parser.add_argument("--audit", action="store_true")
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        freeze = _verify_freeze()
        if args.verify_freeze and not args.audit:
            print(json.dumps({"status": "freeze-verified", "raw_reads": 0, "freeze_sha256": FREEZE_SHA256}, sort_keys=True))
            return 0
        if not args.audit:
            parser.error("choose --verify-freeze or --audit")
        manifest = _verify_manifest(freeze)
        result = _audit(freeze, manifest, args.output_root)
        print(json.dumps({"status": result["status"], "raw_reads": result["read_accounting"]["raw_device_gnss_read_count_total"], "all_gates_passed": result["gates"]["all_passed"]}, sort_keys=True))
        return 0
    except Phase58Error as exc:
        print(json.dumps({"status": "fail-closed", "error": str(exc), "raw_reads": 0}, sort_keys=True))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
