#!/usr/bin/env python3
"""Truth-free Phase61 C/N0 pseudorange CMC calibration audit.

The command reads each of the four frozen Pixel5 Android ``device_gnss.csv``
files once in one process.  It reconstructs raw Android-clock pseudorange,
forms ``P-ADR`` on continuous, already-valid code/carrier arcs, removes the
arc median, and audits the resulting absolute residual against ``Cn0DbHz``.
No truth, navigation, solver, coordinates, IMU, enriched pseudorange, or
previous phase metric payload is opened.  The source-supported candidate shape
is the fixed 20-dB exponential; alpha is a robust scale estimated only in
route-leave-one-out training data.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import math
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_freeze_v1.json"
FREEZE_SHA256 = "d80122d87f2f6e4529483e0f319951ea1f199e46120bd0e785a2adb5eac9685d"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase61_pixel5_cn0_pseudorange_calibration_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase61-pixel5-cn0-pseudorange-calibration-v1"

SCHEMA = "smartphone-r5-phase61-pixel5-cn0-pseudorange-calibration.v1"
FREEZE_SCHEMA = "smartphone-r5-phase61-pixel5-cn0-pseudorange-calibration-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase61-pixel5-cn0-pseudorange-calibration-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = 299_792_458.0
CN0_REFERENCE_DBHZ = 40.0
CN0_BIN_LABELS = ("20_to_25", "25_to_30", "30_to_35", "35_to_40", ">=40")
CN0_BIN_EDGES = (20.0, 25.0, 30.0, 35.0, 40.0)
TRANSITION_MAX_NS = 1_500_000_000
MIN_P = 10_000_000.0
MAX_P = 40_000_000.0


class Phase61Error(ValueError):
    """Raised when the immutable Phase61 contract is violated."""


def _fail(message: str) -> Phase61Error:
    return Phase61Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "kaggle",
        "token", "device_wls", "precomputed", "svposition", "svelevation",
        "coordinates", "device_imu", "navigation", "solver", "trajectory",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase61 path: {path}")


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


def _load_json_once(path: Path, label: str, expected_sha256: str | None = None) -> tuple[dict[str, Any], str]:
    payload, digest = _read_bytes_once(path, label, expected_sha256)
    try:
        import json
        value = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, ValueError) as exc:
        raise _fail(f"invalid {label}") from exc
    if not isinstance(value, dict):
        raise _fail(f"{label} must be an object")
    return value, digest


def _json_bytes(value: Any) -> bytes:
    import json
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode()


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with open(descriptor, "wb", closefd=True) as handle:
            handle.write(payload)
            handle.flush()
            import os
            os.fsync(handle.fileno())
        import os
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
    data = [float(value) for value in values if _finite(value)]
    return float(statistics.median(data)) if data else 0.0


def _percentile(values: Iterable[float], fraction: float) -> float:
    data = sorted(float(value) for value in values if _finite(value))
    if not data:
        return 0.0
    rank = fraction * (len(data) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return data[lower]
    return data[lower] + (rank - lower) * (data[upper] - data[lower])


def _mad(values: Iterable[float], center: float | None = None) -> float:
    data = [float(value) for value in values if _finite(value)]
    if not data:
        return 0.0
    actual = _median(data) if center is None else float(center)
    return _median(abs(value - actual) for value in data)


def _distribution(values: Iterable[float], center: float | None = None) -> dict[str, Any]:
    data = [float(value) for value in values if _finite(value)]
    if not data:
        return {"count": 0, "median": 0.0, "mad": 0.0, "p50_abs": 0.0, "p95_abs": 0.0, "max_abs": 0.0}
    actual = _median(data) if center is None else float(center)
    absolute = [abs(value - actual) for value in data]
    return {
        "count": len(data),
        "median": _median(data),
        "mad": _mad(data, actual),
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


def _shape(cn0_dbhz: float) -> float:
    """Fixed source-supported 20-dB C/N0 shape."""
    if not _finite(cn0_dbhz):
        return math.nan
    value = 10.0 ** (-(float(cn0_dbhz) - CN0_REFERENCE_DBHZ) / 20.0)
    return value if _finite(value) and value > 0.0 else math.nan


def _cn0_bin(value: float | None) -> str:
    if value is None or not _finite(value) or float(value) < CN0_BIN_EDGES[0]:
        return "missing_or_below_20"
    for index, label in enumerate(CN0_BIN_LABELS[:-1]):
        if float(value) < CN0_BIN_EDGES[index + 1]:
            return label
    return CN0_BIN_LABELS[-1]


def _fit_alpha(items: Sequence[dict[str, Any]]) -> float:
    ratios: list[float] = []
    for item in items:
        cn0 = item.get("cn0_dbhz")
        residual = item.get("abs_centered_cmc_m")
        if cn0 is None or float(cn0) < 20.0 or not _finite(cn0) or not _finite(residual):
            continue
        shape = _shape(float(cn0))
        if _finite(shape) and shape > 0.0:
            ratios.append(float(residual) / shape)
    alpha = _median(ratios)
    return alpha if _finite(alpha) and alpha > 0.0 else math.nan


def _candidate_sigma(cn0_dbhz: float | None, alpha_m: float) -> float | None:
    if cn0_dbhz is None or not _finite(cn0_dbhz) or not _finite(alpha_m) or alpha_m <= 0.0:
        return None
    value = alpha_m * _shape(float(cn0_dbhz))
    return value if _finite(value) and value > 0.0 else None


def _signal_family(system: str, signal: str) -> str:
    return f"{system}:{signal}"


def _prepare_parser() -> Any:
    """Load only the reviewed raw parser modules; neither reads raw files."""
    benchmark_dir = str(ROOT / "apps/commands/benchmarks")
    if benchmark_dir not in sys.path:
        sys.path.insert(0, benchmark_dir)
    # Phase55 parser carries the ADR-rich row contract needed for CMC.  The
    # Phase48 source is independently pinned in the freeze as the reviewed
    # Phase25 raw code/rate parsing contract; no Phase47 result is imported.
    import gnss_smartphone_phase55_pixel5_adr_uncertainty_audit as raw_parser  # noqa: E402
    return raw_parser


def _source_contract(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        payload, digest = _read_bytes_once(ROOT / pin["path"], f"Phase61 static source {name}", pin["sha256"])
        try:
            contents[name] = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise _fail(f"Phase61 source is not UTF-8: {name}") from exc
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"]
    observation = contents["observation_header"]
    fgo = contents["fgo_problems_cpp"]
    config = contents["fgo_config_header"]
    cli = contents["native_fgo_cli"]
    upstream = contents["observable_upstream_preprocessing"]
    adapter_lower = adapter.lower()
    observation_lower = observation.lower()
    fgo_lower = fgo.lower()
    config_lower = config.lower()
    cli_lower = cli.lower()
    upstream_lower = upstream.lower()
    return {
        "source_hashes": hashes,
        "adapter": {
            "parses_cn0_dbhz": "cn0dbhz" in adapter_lower,
            "parses_adr_state_and_meters": "accumulateddeltarangestate" in adapter_lower and "accumulateddeltarangemeters" in adapter_lower,
            "applies_existing_multipath_mask": "multipath_indicator == 1" in adapter_lower or "multipath_indicator == 1" in adapter_lower,
            "applies_existing_low_cn0_mask": "cn0_dbhz < 20.0" in adapter_lower,
            "uses_raw_clock_pseudorange": "rawpseudorange" in adapter_lower and "full_bias_nanos" in adapter_lower,
        },
        "observation": {
            "retains_cn0_as_snr": "double snr" in observation_lower,
            "retains_pseudorange": "double pseudorange" in observation_lower,
            "retains_carrier_phase": "double carrier_phase" in observation_lower,
            "candidate_cn0_pseudorange_model_absent": "native_cn0_pseudorange" not in observation_lower,
        },
        "fgo": {
            "adopts_pseudorange_factors": "has_pseudorange" in fgo_lower and "pseudorange_sigma_m" in fgo_lower,
            "legacy_elevation_sigma_path": "pseudorange_sigma_m /" in fgo_lower and "sin(" in fgo_lower,
            "upstream_quality_switch": "use_upstream_observable_quality" in fgo_lower,
            "uses_snr_only_when_upstream_enabled": "use_upstream_observable_quality" in fgo_lower and "snrpercentilesigma" in fgo_lower,
            "candidate_cn0_pseudorange_model_absent": "native_cn0_pseudorange" not in fgo_lower and "cn0_pseudorange_calibration" not in fgo_lower,
        },
        "config": {
            "pseudorange_sigma_default_3m": "double pseudorange_sigma_m = 3.0" in config_lower,
            "elevation_sigma_power_default_1": "double pseudorange_elevation_sigma_power = 1.0" in config_lower,
            "upstream_quality_default_false": "bool use_upstream_observable_quality = false" in config_lower,
        },
        "cli": {
            "candidate_cn0_pseudorange_option_absent": "cn0-pseudorange" not in cli_lower,
            "upstream_quality_option_present": "upstream-observable-quality" in cli_lower,
        },
        "upstream_contract": {
            "twenty_db_exponent": "pow(10.0, -(snr_dbhz - percentile85_dbhz) / 20.0)" in upstream,
            "pseudorange_snr_path_present": "case 'p': return scale * factor" in upstream_lower,
        },
    }


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase61 freeze", FREEZE_SHA256)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase61 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase61-raw-read":
        raise _fail("Phase61 freeze schema/status mismatch")
    cohort = freeze.get("cohort", {})
    if cohort.get("route_order") != list(ROUTES) or cohort.get("route_disjoint") is not True:
        raise _fail("Phase61 route order/disjoint contract changed")
    inputs = freeze.get("exact_raw_inputs")
    if not isinstance(inputs, dict) or tuple(inputs) != ROUTES:
        raise _fail("Phase61 exact raw input map changed")
    for route in ROUTES:
        pin = inputs[route]
        if len(str(pin.get("sha256", ""))) != 64 or int(pin.get("file_size", 0)) <= 0:
            raise _fail(f"Phase61 raw hash/size missing for {route}")
    policy = freeze.get("input_policy", {})
    expected_policy = {
        "single_evaluator_process": True,
        "raw_device_gnss_read_count_per_route": 1,
        "raw_device_gnss_read_count_total": 4,
        "truth_read_count": 0,
        "phase47_metric_payload_reads": 0,
        "phase60_metric_payload_reads": 0,
        "phase43_metric_payload_reads": 0,
        "brdc_nav_read_count": 0,
        "solver_or_trajectory_reruns": 0,
        "raw_device_imu_read_count": 0,
        "coordinate_or_wls_inputs": 0,
        "SvPosition_or_SvElevation": 0,
        "enriched_RawPseudorange_inputs": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerialization_count": 0,
        "kaggle_or_token_access": 0,
        "correction_fit_or_application_before_gate": False,
    }
    if any(policy.get(key) != value for key, value in expected_policy.items()):
        raise _fail("Phase61 read policy changed")
    gates = freeze.get("numeric_gates", {})
    expected_gates = {
        ("coverage", "min_cmc_rows_per_route"): 5000,
        ("coverage", "min_distinct_satellites_per_route"): 5,
        ("coverage", "min_finite_positive_cn0_fraction"): 0.80,
        ("coverage", "min_populated_cn0_bins_per_route"): 4,
        ("coverage", "min_rows_per_populated_cn0_bin"): 100,
        ("coverage", "min_valid_rows_per_adr_arc"): 2,
        ("coverage", "min_signal_families_per_route"): 2,
        ("coverage", "raw_domain_coverage_required"): 1.0,
        ("routewise_monotonicity", "spearman_cn0_vs_abs_centered_cmc_max"): -0.25,
        ("loo_calibration", "fold_count"): 4,
        ("loo_calibration", "fit_routes_per_fold"): 3,
        ("loo_calibration", "alpha_max_over_min_ratio_max"): 3.0,
        ("loo_calibration", "alpha_coefficient_of_variation_max"): 0.50,
        ("non_common_and_composition", "min_satellite_groups_per_route"): 3,
        ("non_common_and_composition", "min_rows_per_satellite_group"): 100,
        ("non_common_and_composition", "satellite_group_direction_fraction_min"): 0.67,
        ("non_common_and_composition", "satellite_group_spearman_max"): -0.10,
        ("non_common_and_composition", "min_signal_groups_per_route"): 2,
        ("non_common_and_composition", "min_rows_per_signal_group"): 100,
        ("non_common_and_composition", "signal_group_direction_fraction_min"): 1.0,
        ("non_common_and_composition", "signal_group_spearman_max"): -0.15,
        ("current_factor_impact_proxy", "configured_base_sigma_m"): 3.0,
        ("current_factor_impact_proxy", "candidate_above_configured_base_fraction_min"): 0.10,
        ("current_factor_impact_proxy", "candidate_sigma_excess_p95_m_min"): 0.50,
    }
    for (section, key), expected in expected_gates.items():
        if gates.get(section, {}).get(key) != expected:
            raise _fail(f"Phase61 numeric gate changed: {section}.{key}")
    if freeze.get("cn0_bins", {}).get("edges_dbhz") != list(CN0_BIN_EDGES):
        raise _fail("Phase61 C/N0 bin edges changed")
    if freeze.get("cn0_bins", {}).get("labels") != list(CN0_BIN_LABELS):
        raise _fail("Phase61 C/N0 bin labels changed")
    if gates.get("routewise_monotonicity", {}).get("allow_exact_equality") is not True:
        raise _fail("Phase61 monotonicity equality policy changed")
    if gates.get("presentation_integrity", {}).get("aggregate_recomputed_exact") is not True:
        raise _fail("Phase61 presentation policy changed")
    fixed = freeze.get("fixed_observable", {})
    if fixed.get("cmc") != "cmc_m = P_raw_m - AccumulatedDeltaRangeMeters_signed_m":
        raise _fail("Phase61 CMC sign changed")
    if fixed.get("minimum_arc_length") != 2 or "singleton" not in str(fixed.get("singleton_arc_policy", "")).lower():
        raise _fail("Phase61 singleton arc contract changed")
    if fixed.get("candidate_model") != "sigma_model_m = alpha_m * 10^(-(Cn0DbHz-40.0)/20.0)":
        raise _fail("Phase61 candidate model changed")
    pre = freeze.get("pre_read_assertions")
    if not isinstance(pre, dict) or any(value is not False for value in pre.values()):
        raise _fail("Phase61 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase61 evaluator manifest")
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase61 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase61-raw-read":
        raise _fail("Phase61 manifest schema/status mismatch")
    pin = manifest.get("freeze", {})
    if pin.get("path") != _relative(FREEZE) or pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase61 manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("single_process") is not True or evaluator.get("native_solver_invoked") is not False or evaluator.get("raw_reads_per_route") != 1:
        raise _fail("Phase61 manifest permits forbidden execution")
    if manifest.get("cohort", {}).get("route_order") != list(ROUTES):
        raise _fail("Phase61 manifest route order mismatch")
    required_forbidden = (
        "ground_truth.csv", "navigation", "solver/trajectory", "coordinates", "MAT",
        "Phase47 metric payload", "Phase60 metric payload", "native correction",
    )
    forbidden = manifest.get("forbidden", [])
    if not isinstance(forbidden, list) or not all(item in forbidden for item in required_forbidden):
        raise _fail("Phase61 forbidden policy missing")
    for name in ("source", "test", "cmake"):
        file_pin = evaluator.get(name, {})
        file_path = ROOT / str(file_pin.get("path", ""))
        if not file_path.is_file() or len(str(file_pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase61 {name} pin missing")
        if _sha256_bytes(file_path.read_bytes()) != file_pin["sha256"]:
            raise _fail(f"Phase61 {name} hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _valid_cmc_row(row: Any, parser: Any) -> bool:
    return (
        row.pseudorange_m is not None and _finite(row.pseudorange_m)
        and MIN_P <= float(row.pseudorange_m) <= MAX_P
        and row.adr_m is not None and _finite(row.adr_m)
        and parser._adr_unflagged(row)
        and not bool(row.code_masked)
    )


def _cmc_rows(rows: Sequence[Any], parser: Any) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    """Build arc-median-centered CMC rows without reusing Phase47 metrics."""
    grouped: dict[tuple[str, int, str], list[Any]] = defaultdict(list)
    for row in rows:
        grouped[row.sat_signal].append(row)
    arc_records: dict[tuple[str, int, str, int], list[dict[str, Any]]] = defaultdict(list)
    events: list[dict[str, Any]] = []
    valid_before_arc = 0
    singleton_rows = 0
    arc_count = 0
    for key, sequence in sorted(grouped.items()):
        sequence.sort(key=lambda row: (row.utc_ms, row.row_number))
        previous: Any | None = None
        previous_valid = False
        arc = -1
        for row in sequence:
            current_valid = _valid_cmc_row(row, parser)
            dt_ns = None if previous is None else int(row.time_ns) - int(previous.time_ns)
            hcdc_change = previous is not None and int(row.hcdc) != int(previous.hcdc)
            segment_change = previous is not None and int(row.segment) != int(previous.segment)
            gap = dt_ns is not None and (dt_ns <= 0 or dt_ns > TRANSITION_MAX_NS)
            adr_boundary = row.adr_state is None or not parser._adr_unflagged(row)
            boundary = previous is None or not previous_valid or not current_valid or hcdc_change or segment_change or bool(gap)
            if boundary:
                arc += 1
                arc_count += 1
            if hcdc_change:
                events.append({"kind": "hcdc_arc_boundary", "system": key[0], "svid": key[1], "signal": key[2], "utcTimeMillis": int(row.utc_ms), "arc": arc})
            if segment_change:
                events.append({"kind": "raw_clock_segment_boundary", "system": key[0], "svid": key[1], "signal": key[2], "utcTimeMillis": int(row.utc_ms), "arc": arc})
            if gap:
                events.append({"kind": "time_or_order_arc_boundary", "system": key[0], "svid": key[1], "signal": key[2], "utcTimeMillis": int(row.utc_ms), "dt_ns": dt_ns, "arc": arc})
            if adr_boundary and current_valid is False and row.adr_state is not None:
                events.append({"kind": "adr_validity_boundary", "system": key[0], "svid": key[1], "signal": key[2], "utcTimeMillis": int(row.utc_ms), "adr_state": int(row.adr_state), "arc": arc})
            if current_valid:
                valid_before_arc += 1
                cmc_value = float(row.pseudorange_m) - float(row.adr_m)
                arc_records[(key[0], int(key[1]), key[2], arc)].append({
                    "utcTimeMillis": int(row.utc_ms),
                    "row_number": int(row.row_number),
                    "system": key[0],
                    "svid": int(key[1]),
                    "signal": key[2],
                    "arc": arc,
                    "cmc_m": cmc_value,
                    "cn0_dbhz": float(row.cn0) if row.cn0 is not None and _finite(row.cn0) else None,
                    "hcdc": int(row.hcdc),
                    "segment": int(row.segment),
                    "multipath": int(row.multipath) if row.multipath is not None else None,
                    "state": int(row.state) if row.state is not None else None,
                    "mask_reasons": list(getattr(row, "mask_reasons", ())),
                })
            previous, previous_valid = row, current_valid
    eligible: list[dict[str, Any]] = []
    eligible_arc_lengths: list[int] = []
    for arc_key, arc_items in sorted(arc_records.items()):
        if len(arc_items) < 2:
            singleton_rows += len(arc_items)
            continue
        center = _median(item["cmc_m"] for item in arc_items)
        eligible_arc_lengths.append(len(arc_items))
        for item in arc_items:
            centered = float(item["cmc_m"]) - center
            output = dict(item)
            output["arc_median_cmc_m"] = center
            output["centered_cmc_m"] = centered
            output["abs_centered_cmc_m"] = abs(centered)
            eligible.append(output)
    return eligible, events, {
        "valid_rows_before_arc_length": valid_before_arc,
        "singleton_arc_rows_excluded": singleton_rows,
        "arc_count": arc_count,
        "eligible_arc_count": len(eligible_arc_lengths),
        "eligible_arc_min_rows": min(eligible_arc_lengths) if eligible_arc_lengths else 0,
        "eligible_arc_max_rows": max(eligible_arc_lengths) if eligible_arc_lengths else 0,
        "eligible_arc_median_rows": _median(eligible_arc_lengths),
    }


def _bin_summary(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[str, list[float]] = {label: [] for label in CN0_BIN_LABELS}
    for item in items:
        label = _cn0_bin(item.get("cn0_dbhz"))
        if label in grouped and _finite(item.get("abs_centered_cmc_m")):
            grouped[label].append(float(item["abs_centered_cmc_m"]))
    ordered: dict[str, Any] = {}
    for label in CN0_BIN_LABELS:
        values = grouped[label]
        ordered[label] = {
            "count": len(values),
            "residual_m": _distribution(values, 0.0),
            "median_abs_m": _median(values),
            "p95_abs_m": _percentile(values, 0.95),
            "max_abs_m": max(values) if values else 0.0,
        }
    populated = [label for label in CN0_BIN_LABELS if grouped[label]]
    medians = [ordered[label]["median_abs_m"] for label in populated]
    p95s = [ordered[label]["p95_abs_m"] for label in populated]
    return {
        "ordered": ordered,
        "populated_labels": populated,
        "populated_count": len(populated),
        "median_non_increasing": all(left >= right for left, right in zip(medians, medians[1:])),
        "p95_non_increasing": all(left >= right for left, right in zip(p95s, p95s[1:])),
    }


def _group_relation(items: Sequence[dict[str, Any]], key_fn: Any, min_rows: int) -> dict[str, Any]:
    """Summarize each group independently (never reuse a stale loop value)."""
    groups: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for item in items:
        groups[str(key_fn(item))].append(item)
    report: dict[str, Any] = {}
    for name in sorted(groups):
        group = groups[name]
        finite = [item for item in group if item.get("cn0_dbhz") is not None and _finite(item.get("cn0_dbhz")) and float(item["cn0_dbhz"]) >= 20.0 and _finite(item.get("abs_centered_cmc_m"))]
        x = [float(item["cn0_dbhz"]) for item in finite]
        y = [float(item["abs_centered_cmc_m"]) for item in finite]
        report[name] = {
            "count": len(group),
            "finite_cn0_count": len(finite),
            "spearman_cn0_vs_abs_centered_cmc": _spearman(x, y),
            "cn0_dbhz": _distribution(x),
            "abs_centered_cmc_m": _distribution(y, 0.0),
            "meets_min_rows": len(group) >= min_rows,
        }
    return report


def _same_epoch_noncommon(items: Sequence[dict[str, Any]]) -> dict[str, Any]:
    grouped: dict[tuple[int, int], list[float]] = defaultdict(list)
    for item in items:
        grouped[(int(item["utcTimeMillis"]), int(item["hcdc"]))].append(float(item["centered_cmc_m"]))
    centered_abs: list[float] = []
    spreads: list[float] = []
    epoch_count = 0
    for values in grouped.values():
        if len(values) < 2:
            continue
        epoch_count += 1
        center = _median(values)
        centered = [value - center for value in values]
        centered_abs.extend(abs(value) for value in centered)
        spreads.append(max(values) - min(values))
    return {
        "comparable_epoch_group_count": epoch_count,
        "centered_cmc_abs_m": _distribution(centered_abs, 0.0),
        "within_epoch_spread_m": _distribution(spreads, 0.0),
        "centered_cmc_p50_min_gate_m": _percentile(centered_abs, 0.50),
    }


def _calibration(items: Sequence[dict[str, Any]], alpha: float) -> dict[str, Any]:
    normalized: list[float] = []
    model_values: list[float] = []
    excess_values: list[float] = []
    for item in items:
        model = _candidate_sigma(item.get("cn0_dbhz"), alpha)
        if model is None or not _finite(item.get("abs_centered_cmc_m")):
            continue
        residual = float(item["abs_centered_cmc_m"])
        normalized.append(residual / model)
        model_values.append(model)
        if model > 3.0:
            excess_values.append(model - 3.0)
    return {
        "alpha_m_at_40_dbhz": alpha if _finite(alpha) else None,
        "candidate_sigma_m": _distribution(model_values, 0.0),
        "normalized_abs_centered_cmc": _distribution(normalized, 0.0),
        "normalized_median": _median(normalized),
        "normalized_le_one_fraction": sum(value <= 1.0 for value in normalized) / len(normalized) if normalized else 0.0,
        "candidate_above_configured_base_count": len(excess_values),
        "candidate_above_configured_base_fraction": len(excess_values) / len(model_values) if model_values else 0.0,
        "candidate_sigma_excess_p95_m": _percentile(excess_values, 0.95) if excess_values else 0.0,
    }


def _route_report(route: str, rows: Sequence[Any], metadata: dict[str, Any], path: Path, digest: str, parser: Any) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    parser._assign_segments(rows)
    items, events, arc_info = _cmc_rows(rows, parser)
    finite_cn0 = [item for item in items if item.get("cn0_dbhz") is not None and _finite(item.get("cn0_dbhz")) and float(item["cn0_dbhz"]) >= 20.0]
    bins = _bin_summary(items)
    satellites = sorted({(item["system"], int(item["svid"])) for item in items})
    signals = sorted({_signal_family(item["system"], item["signal"]) for item in items})
    sat_groups = _group_relation(items, lambda item: f"{item['system']}:{item['svid']}", 100)
    signal_groups = _group_relation(items, lambda item: _signal_family(item["system"], item["signal"]), 100)
    x = [float(item["cn0_dbhz"]) for item in finite_cn0]
    y = [float(item["abs_centered_cmc_m"]) for item in finite_cn0]
    report = {
        "route": route,
        "input": {"path": _relative(path), "bytes": int(path.stat().st_size), "sha256": digest},
        "headers": {
            "columns": metadata.get("header_columns", []),
            "optional_field_presence": metadata.get("optional_field_presence", {}),
        },
        "rows": {
            "raw": int(metadata.get("raw_rows", 0)),
            "non_raw": int(metadata.get("non_raw_rows", 0)),
            "unsupported_signal_rows_informational": int(metadata.get("unsupported_signal_rows", 0)),
            "epochs": int(metadata.get("epoch_count", 0)),
            "raw_domain_coverage": 1.0,
            "repeated_epoch_key_count": int(metadata.get("repeated_epoch_key_count", 0)),
            "nonmonotonic_epoch_key_count": int(metadata.get("nonmonotonic_epoch_key_count", 0)),
            "valid_rows_before_arc_length": int(arc_info["valid_rows_before_arc_length"]),
            "singleton_arc_rows_excluded": int(arc_info["singleton_arc_rows_excluded"]),
            "cmc_rows": len(items),
            "finite_positive_cn0_count": len(finite_cn0),
            "finite_positive_cn0_fraction": len(finite_cn0) / len(items) if items else 0.0,
            "distinct_satellite_count": len(satellites),
            "satellites": [f"{system}:{svid}" for system, svid in satellites],
            "signal_families": signals,
        },
        "arc_contract": {
            "arc_count": int(arc_info["arc_count"]),
            "eligible_arc_count": int(arc_info["eligible_arc_count"]),
            "eligible_arc_min_rows": int(arc_info["eligible_arc_min_rows"]),
            "eligible_arc_median_rows": arc_info["eligible_arc_median_rows"],
            "eligible_arc_max_rows": int(arc_info["eligible_arc_max_rows"]),
        },
        "cmc_residual": {
            "raw_cmc_m": _distribution([float(item["cmc_m"]) for item in items]),
            "centered_cmc_m": _distribution([float(item["centered_cmc_m"]) for item in items], 0.0),
            "abs_centered_cmc_m": _distribution([float(item["abs_centered_cmc_m"]) for item in items], 0.0),
            "cn0_spearman": _spearman(x, y),
            "finite_cn0_count": len(finite_cn0),
        },
        "cn0_bins": bins,
        "groups": {
            "satellite": sat_groups,
            "signal": signal_groups,
            "satellite_group_count": len(sat_groups),
            "signal_group_count": len(signal_groups),
        },
        "same_epoch_noncommon": _same_epoch_noncommon(items),
        "events": {"count": len(events)},
        "_items": items,
        "_events": events,
    }
    return report, events


def _loo(reports: dict[str, dict[str, Any]]) -> dict[str, Any]:
    folds: list[dict[str, Any]] = []
    alphas: list[float] = []
    for omitted in ROUTES:
        training = [item for route, report in reports.items() if route != omitted for item in report["_items"]]
        heldout = reports[omitted]["_items"]
        alpha = _fit_alpha(training)
        alphas.append(alpha)
        calibration = _calibration(heldout, alpha)
        finite = [item for item in heldout if item.get("cn0_dbhz") is not None and _finite(item.get("cn0_dbhz")) and float(item["cn0_dbhz"]) >= 20.0]
        x = [float(item["cn0_dbhz"]) for item in finite]
        y = [float(item["abs_centered_cmc_m"]) for item in finite]
        rho = _spearman(x, y)
        folds.append({
            "omitted_route": omitted,
            "training_routes": [route for route in ROUTES if route != omitted],
            "training_cmc_count": len(training),
            "heldout_cmc_count": len(heldout),
            "alpha_m_at_40_dbhz": alpha if _finite(alpha) else None,
            "heldout_cn0_spearman": rho,
            "heldout_normalized_median": calibration["normalized_median"],
            "heldout_normalized_le_one_fraction": calibration["normalized_le_one_fraction"],
            "heldout_candidate_above_base_fraction": calibration["candidate_above_configured_base_fraction"],
            "heldout_candidate_sigma_excess_p95_m": calibration["candidate_sigma_excess_p95_m"],
            "direction_negative": rho <= -0.25,
        })
    finite_alphas = [float(alpha) for alpha in alphas if _finite(alpha) and float(alpha) > 0.0]
    alpha_ratio = max(finite_alphas) / min(finite_alphas) if finite_alphas else math.inf
    alpha_mean = statistics.fmean(finite_alphas) if finite_alphas else math.nan
    alpha_cv = statistics.pstdev(finite_alphas) / alpha_mean if finite_alphas and alpha_mean > 0.0 else math.inf
    return {
        "folds": folds,
        "alpha_values_m_at_40_dbhz": [alpha if _finite(alpha) else None for alpha in alphas],
        "alpha_max_over_min_ratio": alpha_ratio if _finite(alpha_ratio) else None,
        "alpha_coefficient_of_variation": alpha_cv if _finite(alpha_cv) else None,
        "direction_stable": bool(folds) and all(fold["direction_negative"] for fold in folds),
    }


def _presentation_integrity(reports: dict[str, dict[str, Any]], aggregate: dict[str, Any], loo: dict[str, Any]) -> dict[str, Any]:
    route_checks: dict[str, Any] = {}
    for route, report in reports.items():
        items = report["_items"]
        finite_count = int(report["cmc_residual"]["finite_cn0_count"])
        bins = report["cn0_bins"]["ordered"]
        bin_sum = sum(int(value["count"]) for value in bins.values())
        sat_groups = report["groups"]["satellite"]
        signal_groups = report["groups"]["signal"]
        route_checks[route] = {
            "five_bin_order": tuple(bins) == CN0_BIN_LABELS,
            "bin_counts_sum_finite_cn0": bin_sum == finite_count,
            "satellite_group_counts_sum_cmc": sum(int(value["count"]) for value in sat_groups.values()) == len(items),
            "signal_group_counts_sum_cmc": sum(int(value["count"]) for value in signal_groups.values()) == len(items),
            "group_stats_match_counts": all(int(value["abs_centered_cmc_m"]["count"]) == int(value["finite_cn0_count"]) for value in list(sat_groups.values()) + list(signal_groups.values())),
        }
    medians = aggregate.get("route_median_abs_centered_cmc_m", {})
    retained = tuple(medians.keys()) == ROUTES and len(medians) == 4
    expected_median = _median(medians.values())
    expected_mad = _mad(medians.values())
    return {
        "all_route_group_counts_exact": all(all(value.values()) for value in route_checks.values()),
        "route_checks": route_checks,
        "four_route_medians_retained_in_order": retained,
        "aggregate_recomputed_exact": aggregate.get("aggregate_median_m") == expected_median and aggregate.get("aggregate_mad_m") == expected_mad,
        "four_loo_folds_retained": len(loo.get("folds", [])) == 4 and tuple(fold.get("omitted_route") for fold in loo.get("folds", [])) == ROUTES,
        "no_stale_group_loop_state": all(len(report["groups"]["signal"]) == int(report["groups"]["signal_group_count"]) for report in reports.values()),
        "no_collapsed_route_aggregate": retained,
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
        raise _fail(f"Phase61 output already exists: {output_root}")
    parser = _prepare_parser()
    source = _source_contract(freeze)
    reports: dict[str, dict[str, Any]] = {}
    all_events: list[dict[str, Any]] = []
    reads: dict[str, Any] = {
        "single_process": True,
        "raw_device_gnss_reads": {},
        "raw_device_gnss_read_count_total": 0,
        "truth_reads": 0,
        "phase47_metric_payload_reads": 0,
        "phase60_metric_payload_reads": 0,
        "phase43_metric_payload_reads": 0,
        "navigation_reads": 0,
        "brdc_nav_reads": 0,
        "solver_or_trajectory_reruns": 0,
        "raw_device_imu_reads": 0,
        "coordinate_or_wls_inputs": 0,
        "SvPosition_or_SvElevation": 0,
        "enriched_RawPseudorange_inputs": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerializations": 0,
        "kaggle_or_token_access": 0,
        "correction_fit_or_application": 0,
        "source_static_reads": len(source["source_hashes"]),
    }
    for route in ROUTES:
        pin = freeze["exact_raw_inputs"][route]
        path = ROOT / pin["path"]
        payload, digest = _read_bytes_once(path, f"Phase61 raw device_gnss {route}", pin["sha256"], int(pin["file_size"]))
        rows, metadata = parser._parse_payload(payload)
        del payload
        report, events = _route_report(route, rows, metadata, path, digest, parser)
        reports[route] = report
        for event in events:
            event["route"] = route
        all_events.extend(events)
        reads["raw_device_gnss_reads"][route] = 1
        reads["raw_device_gnss_read_count_total"] += 1
    medians = {route: _median(item["abs_centered_cmc_m"] for item in report["_items"]) for route, report in reports.items()}
    aggregate = {
        "route_count": len(reports),
        "route_median_abs_centered_cmc_m": medians,
        "aggregate_median_m": _median(medians.values()),
        "aggregate_mad_m": _mad(medians.values()),
        "pairwise_route_median_distances_m": [
            {"route_a": left, "route_b": right, "distance_m": abs(medians[left] - medians[right])}
            for index, left in enumerate(ROUTES) for right in ROUTES[index + 1:]
        ],
    }
    loo = _loo(reports)
    integrity = _presentation_integrity(reports, aggregate, loo)
    coverage = {
        "min_cmc_rows": min((int(report["rows"]["cmc_rows"]) for report in reports.values()), default=0),
        "min_satellites": min((int(report["rows"]["distinct_satellite_count"]) for report in reports.values()), default=0),
        "min_cn0_fraction": min((float(report["rows"]["finite_positive_cn0_fraction"]) for report in reports.values()), default=0.0),
        "min_populated_cn0_bins": min((int(report["cn0_bins"]["populated_count"]) for report in reports.values()), default=0),
        "min_populated_bin_rows": min((min((int(value["count"]) for value in report["cn0_bins"]["ordered"].values() if int(value["count"]) > 0), default=0) for report in reports.values()), default=0),
        "min_signal_families": min((len(report["rows"]["signal_families"]) for report in reports.values()), default=0),
        "min_arc_rows": min((int(report["arc_contract"]["eligible_arc_min_rows"]) for report in reports.values()), default=0),
    }
    routewise_pass = all(
        float(report["cmc_residual"]["cn0_spearman"]) <= -0.25
        and bool(report["cn0_bins"]["median_non_increasing"])
        and bool(report["cn0_bins"]["p95_non_increasing"])
        for report in reports.values()
    )
    loo_pass = (
        all(0.25 <= float(fold["heldout_normalized_median"]) <= 4.0 and 0.05 <= float(fold["heldout_normalized_le_one_fraction"]) <= 0.95 for fold in loo["folds"])
        and loo["alpha_max_over_min_ratio"] is not None and float(loo["alpha_max_over_min_ratio"]) <= 3.0
        and loo["alpha_coefficient_of_variation"] is not None and float(loo["alpha_coefficient_of_variation"]) <= 0.50
        and bool(loo["direction_stable"])
    )
    satellite_pass = True
    signal_pass = True
    for report in reports.values():
        sat = [value for value in report["groups"]["satellite"].values() if value["meets_min_rows"]]
        sat_direction = sum(float(value["spearman_cn0_vs_abs_centered_cmc"]) <= -0.10 for value in sat) / len(sat) if sat else 0.0
        signal = [value for value in report["groups"]["signal"].values() if value["meets_min_rows"]]
        signal_direction = sum(float(value["spearman_cn0_vs_abs_centered_cmc"]) <= -0.15 for value in signal) / len(signal) if signal else 0.0
        satellite_pass = satellite_pass and len(sat) >= 3 and sat_direction >= 0.67
        signal_pass = signal_pass and len(signal) >= 2 and signal_direction >= 1.0
    noncommon_pass = all(float(report["same_epoch_noncommon"]["centered_cmc_p50_min_gate_m"]) >= 0.05 for report in reports.values())
    coverage_pass = (
        coverage["min_cmc_rows"] >= 5000
        and coverage["min_satellites"] >= 5
        and coverage["min_cn0_fraction"] >= 0.80
        and coverage["min_populated_cn0_bins"] >= 4
        and coverage["min_populated_bin_rows"] >= 100
        and coverage["min_arc_rows"] >= 2
        and coverage["min_signal_families"] >= 2
        and all(float(report["rows"]["raw_domain_coverage"]) == 1.0 for report in reports.values())
    )
    impact_loo: dict[str, dict[str, Any]] = {}
    for fold in loo["folds"]:
        route = str(fold["omitted_route"])
        alpha = fold["alpha_m_at_40_dbhz"]
        impact_loo[route] = _calibration(reports[route]["_items"], float(alpha) if alpha is not None else math.nan)
    impact_pass = all(
        float(value["candidate_above_configured_base_fraction"]) >= 0.10
        and float(value["candidate_sigma_excess_p95_m"]) >= 0.50
        for value in impact_loo.values()
    )
    source_pass = all((
        source["adapter"]["parses_cn0_dbhz"],
        source["adapter"]["parses_adr_state_and_meters"],
        source["adapter"]["applies_existing_low_cn0_mask"],
        source["adapter"]["uses_raw_clock_pseudorange"],
        source["observation"]["retains_cn0_as_snr"],
        source["observation"]["retains_pseudorange"],
        source["fgo"]["adopts_pseudorange_factors"],
        source["fgo"]["legacy_elevation_sigma_path"],
        source["fgo"]["upstream_quality_switch"],
        source["fgo"]["uses_snr_only_when_upstream_enabled"],
        source["config"]["pseudorange_sigma_default_3m"],
        source["config"]["elevation_sigma_power_default_1"],
        source["config"]["upstream_quality_default_false"],
        source["fgo"]["candidate_cn0_pseudorange_model_absent"],
        source["cli"]["candidate_cn0_pseudorange_option_absent"],
        source["upstream_contract"]["twenty_db_exponent"],
    ))
    raw_integrity = all(
        report["input"]["sha256"] == freeze["exact_raw_inputs"][route]["sha256"]
        and report["input"]["bytes"] == int(freeze["exact_raw_inputs"][route]["file_size"])
        and int(report["rows"]["repeated_epoch_key_count"]) == 0
        and int(report["rows"]["nonmonotonic_epoch_key_count"]) == 0
        for route, report in reports.items()
    )
    presentation_pass = all(integrity.values())
    gate_rows = {
        "route_count": len(reports) == 4,
        "raw_input_integrity": raw_integrity,
        "source_contract": source_pass,
        "coverage": coverage_pass,
        "routewise_monotonicity": routewise_pass,
        "loo_calibration": loo_pass,
        "per_satellite_non_common_mode": satellite_pass,
        "per_signal_composition": signal_pass,
        "same_epoch_non_common_mode": noncommon_pass,
        "current_factor_impact_proxy": impact_pass,
        "presentation_integrity": presentation_pass,
    }
    all_passed = all(gate_rows.values())
    status = "go-cn0-pseudorange-calibration-audit" if all_passed else "no-go-cn0-pseudorange-calibration-not-identifiable"
    result = {
        "schema_version": SCHEMA,
        "phase": 61,
        "execution_label": "Luna Max",
        "status": status,
        "operation": "truth-free raw Android Cn0DbHz pseudorange CMC residual calibration audit",
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256, "seal_commit": "65a8fb9"},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256, "seal_commit": manifest.get("seal_commit")},
        "evaluator": {"path": _relative(Path(__file__)), "sha256": _sha256_bytes(Path(__file__).read_bytes()), "one_shot": True, "single_process": True, "raw_parser": "Phase55 ADR-rich raw parser; Phase48 raw-clock source hash pinned; no prior metric payload"},
        "source_contract": source,
        "champion_config_metadata": freeze["source_and_factor_audit"]["current_phase43_config"],
        "fixed_observable": freeze["fixed_observable"],
        "routes": {route: _strip_internal(report) for route, report in reports.items()},
        "aggregate": aggregate,
        "loo": loo,
        "candidate_impact_loo": impact_loo,
        "gates": {"all_passed": all_passed, "observed": gate_rows, "minimums": freeze["numeric_gates"], "coverage_observed": coverage},
        "decision": {
            "audit_only": True,
            "native_correction_authorized": False,
            "implementation_stage_authorized": False,
            "strongest_finding": "C/N0-to-arc-centered CMC association is truth-free and is retained with all route/group/LOO diagnostics; any failed gate keeps the candidate diagnostic-only.",
            "next_single_raw_physical_factor_if_no_go": "raw Android pseudorange code tracking / multipath residual calibration with the frozen Cn0 model remains the sole Phase61 factor; preserve Phase43 champion and select no second factor in this phase",
            "phase43_champion_preserved": True,
            "phase51_experimental_preserved": True,
            "phase47_metric_payload_used": False,
            "phase60_metric_payload_used": False,
            "truth_or_metric_payload_used": False,
            "zero_point_782": "not evaluated without truth",
        },
        "read_accounting": reads,
        "events": {"count": len(all_events), "table": "phase61_pixel5_cn0_pseudorange_calibration.events.json"},
        "presentation_integrity": integrity,
        "output_artifacts": {},
    }
    output_root.mkdir(parents=True, exist_ok=False)
    routes_path = output_root / "phase61_pixel5_cn0_pseudorange_calibration.routes.json"
    events_path = output_root / "phase61_pixel5_cn0_pseudorange_calibration.events.json"
    manifest_path = output_root / "phase61_pixel5_cn0_pseudorange_calibration.manifest.json"
    result_path = output_root / "phase61_pixel5_cn0_pseudorange_calibration.json"
    route_bytes = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "routes": result["routes"], "aggregate": aggregate, "loo": loo, "candidate_impact_loo": impact_loo})
    event_bytes = _atomic_json(events_path, {"schema_version": SCHEMA + ".events", "events": all_events})
    result["output_artifacts"] = {
        "routes": {"path": _relative(routes_path), "bytes": len(route_bytes), "sha256": _sha256_bytes(route_bytes)},
        "events": {"path": _relative(events_path), "bytes": len(event_bytes), "sha256": _sha256_bytes(event_bytes)},
    }
    result_bytes = _atomic_json(result_path, result)
    _atomic_json(manifest_path, {"schema_version": SCHEMA + ".manifest", "status": status, "result_sha256": _sha256_bytes(result_bytes), "routes_sha256": _sha256_bytes(route_bytes), "events_sha256": _sha256_bytes(event_bytes), "raw_reads": reads["raw_device_gnss_read_count_total"], "truth_reads": reads["truth_reads"]})
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
            print(_json_bytes({"status": "freeze-verified", "raw_reads": 0, "freeze_sha256": FREEZE_SHA256}).decode().strip())
            return 0
        if not args.audit:
            parser.error("choose --verify-freeze or --audit")
        manifest = _verify_manifest(freeze)
        result = _audit(freeze, manifest, args.output_root)
        print(_json_bytes({"status": result["status"], "raw_reads": result["read_accounting"]["raw_device_gnss_read_count_total"], "all_gates_passed": result["gates"]["all_passed"]}).decode().strip())
        return 0
    except Phase61Error as exc:
        print(_json_bytes({"status": "fail-closed", "error": str(exc), "raw_reads": 0}).decode().strip())
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
