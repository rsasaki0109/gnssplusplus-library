#!/usr/bin/env python3
"""Truth-free Phase60 audit of Android inter-signal bias fields.

The command reads each frozen Pixel5 ``device_gnss.csv`` exactly once in one
process.  It reconstructs only the Phase25 raw-clock pseudorange proxy, then
audits Android's ``FullInterSignalBiasNanos`` and
``SatelliteInterSignalBiasNanos`` availability, source-sign correction,
same-epoch/same-satellite multi-signal consistency, temporal stability, and
common/non-common structure.  It does not open truth, navigation, IMU,
coordinates, enriched pseudorange, solver output, or any prior metric payload.

The Android API defines the two fields independently: Full ISB is the complete
receiver-plus-space-segment inter-signal bias and Satellite ISB is the
space-segment component.  Both use ``corrected pseudorange = raw pseudorange -
field``.  Consequently this audit never adds the fields; ``Full - Satellite``
is reported as the receiver-side remainder and ``Full + Satellite`` is an
explicitly prohibited double correction.
"""

from __future__ import annotations

import argparse
from collections import defaultdict
from dataclasses import dataclass
from decimal import Decimal, InvalidOperation, ROUND_HALF_EVEN, localcontext
import hashlib
import io
import itertools
import json
import math
import os
from pathlib import Path
import statistics
import tempfile
from typing import Any, Iterable, Sequence


ROOT = Path(__file__).resolve().parents[3]
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase60_pixel5_intersignal_bias_freeze_v1.json"
FREEZE_SHA256 = "d4c0a0ea383801b77cd96083a04dd15c79a6061e79be5e70f5e0859d2340e4a0"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase60_pixel5_intersignal_bias_evaluator_manifest_v1.json"
MANIFEST_SHA256 = ""
VERIFIED_MANIFEST_SHA256 = ""
DEFAULT_OUTPUT = ROOT / "output/smartphone-r5/phase60-pixel5-intersignal-bias-v1"

SCHEMA = "smartphone-r5-phase60-pixel5-intersignal-bias.v1"
FREEZE_SCHEMA = "smartphone-r5-phase60-pixel5-intersignal-bias-freeze.v1"
MANIFEST_SCHEMA = "smartphone-r5-phase60-pixel5-intersignal-bias-manifest.v1"

ROUTES = (
    "2021-03-16-18-59-us-ca-mtv-a/pixel5",
    "2021-08-24-20-32-us-ca-mtv-h/pixel5",
    "2022-04-01-18-22-us-ca-lax-t/pixel5",
    "2023-03-08-21-34-us-ca-mtv-u/pixel5",
)

SPEED_OF_LIGHT_MPS = Decimal("299792458")
NANOSECONDS = Decimal("1000000000")
SECONDS_PER_WEEK = Decimal("604800")
HALF_WEEK = Decimal("302400")
SECONDS_PER_DAY = Decimal("86400")
TIME_GAP_NS = 1_000_000_000
MIN_P_M = Decimal("10000000")
MAX_P_M = Decimal("40000000")
BIAS_JUMP_M = 1.0

REQUIRED_COLUMNS = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "BiasNanos",
    "ReceivedSvTimeNanos",
    "Svid",
    "ConstellationType",
    "SignalType",
    "CarrierFrequencyHz",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
)
OPTIONAL_COLUMNS = (
    "State",
    "MultipathIndicator",
    "Cn0DbHz",
    "HardwareClockDiscontinuityCount",
    "TimeOffsetNanos",
    "FullInterSignalBiasNanos",
    "SatelliteInterSignalBiasNanos",
    "FullInterSignalBiasUncertaintyNanos",
    "SatelliteInterSignalBiasUncertaintyNanos",
)

ROUTE_INPUTS = {
    ROUTES[0]: {
        "path": "output/smartphone-r5/phase25-raw-clock-eval-v1/raw/2021-03-16-18-59-us-ca-mtv-a/pixel5/device_gnss.csv",
        "bytes": 57715495,
        "sha256": "c7d50d5127d16586adc6c79d724758e298b385496da22c5e5dfd6ec522cbc863",
    },
    ROUTES[1]: {
        "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2021-08-24-20-32-us-ca-mtv-h/pixel5/inputs/device_gnss.csv",
        "bytes": 74299283,
        "sha256": "46482b82db0992c1f063dbd9cf697268605234d3e38bcbd23525fd4b60bc17a7",
    },
    ROUTES[2]: {
        "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2022-04-01-18-22-us-ca-lax-t/pixel5/inputs/device_gnss.csv",
        "bytes": 33837317,
        "sha256": "50362c01bff3e0bb7088e54021164591cd750227ed97c2fd7d95d763a08798f1",
    },
    ROUTES[3]: {
        "path": "output/smartphone-r5/phase37-pixel5-repeatability-v1/routes/2023-03-08-21-34-us-ca-mtv-u/pixel5/inputs/device_gnss.csv",
        "bytes": 23385520,
        "sha256": "a0fc8e71bdfc03be61b99efcd7d41fbba8ffec126df78b55243f681fd211f204",
    },
}


class Phase60Error(ValueError):
    """Raised when an immutable Phase60 contract is violated."""


def _fail(message: str) -> Phase60Error:
    return Phase60Error(message)


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT))
    except ValueError:
        return str(path)


def _reject_path(path: Path | str) -> None:
    lowered = str(path).lower()
    forbidden = (
        ".mat", "ground_truth", "/truth", "validation", "holdout", "kaggle",
        "token", "archive", "device_imu", "device_wls", "precomputed",
        "svposition", "svelevation", "coordinates", "rawpseudorange",
    )
    if any(token in lowered for token in forbidden):
        raise _fail(f"forbidden Phase60 path: {path}")


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
        raise _fail(f"{label} must be a JSON object")
    return value, digest


def _json_bytes(value: Any) -> bytes:
    return (json.dumps(value, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def _atomic_write(path: Path, payload: bytes) -> None:
    _reject_path(path)
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


def _atomic_json(path: Path, value: Any) -> bytes:
    payload = _json_bytes(value)
    _atomic_write(path, payload)
    return payload


def _finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError, OverflowError):
        return False


def _normalise_header(value: str) -> str:
    return "".join(character.lower() for character in value.strip() if character.isalnum())


def _parse_decimal(token: Any, *, required: bool = True) -> Decimal | None:
    text = "" if token is None else str(token).strip()
    if not text:
        if required:
            raise InvalidOperation
        return None
    try:
        value = Decimal(text)
    except InvalidOperation:
        if required:
            raise
        return None
    if not value.is_finite():
        return None
    return value


def _parse_int(token: Any, *, required: bool = True) -> int | None:
    text = "" if token is None else str(token).strip()
    if not text:
        if required:
            raise ValueError("blank integer")
        return None
    try:
        return int(text, 10)
    except (TypeError, ValueError):
        if required:
            raise
        return None


def _bias_m(value_ns: Decimal | int | float | None) -> float | None:
    """Convert signed Android nanoseconds to metres without changing sign."""
    if value_ns is None:
        return None
    value = value_ns if isinstance(value_ns, Decimal) else Decimal(str(value_ns))
    if not value.is_finite():
        return None
    with localcontext() as context:
        context.prec = 40
        result = value * SPEED_OF_LIGHT_MPS / NANOSECONDS
    value_m = float(result)
    return value_m if math.isfinite(value_m) else None


def _corrected_pseudorange_m(raw_m: float, bias_ns: Decimal | int | float | None) -> float | None:
    bias_m = _bias_m(bias_ns)
    if bias_m is None or not math.isfinite(raw_m):
        return None
    corrected = raw_m - bias_m
    return corrected if math.isfinite(corrected) else None


def _glonass_tow_to_gps(glonass_tow: Decimal, gps_tow_reference: Decimal) -> Decimal:
    day = (gps_tow_reference / SECONDS_PER_DAY).to_integral_value(rounding="ROUND_FLOOR")
    gpst = glonass_tow + day * SECONDS_PER_DAY - Decimal(3 * 3600) + Decimal(18)
    day_offset = day - (gpst / SECONDS_PER_DAY).to_integral_value(rounding="ROUND_FLOOR")
    return gpst + day_offset * SECONDS_PER_DAY


def _raw_pseudorange_m(
    time_ns: int,
    base_full_bias_ns: int,
    bias_ns: Decimal,
    time_offset_ns: Decimal,
    received_sv_time_ns: int,
    constellation: int,
) -> float | None:
    with localcontext() as context:
        context.prec = 50
        clock_ns = Decimal(time_ns - base_full_bias_ns)
        gps_seconds = clock_ns / NANOSECONDS
        if gps_seconds < 0:
            return None
        week = (gps_seconds / SECONDS_PER_WEEK).to_integral_value(rounding="ROUND_FLOOR")
        tow_rx = gps_seconds - week * SECONDS_PER_WEEK - bias_ns / NANOSECONDS
        tow_rx -= time_offset_ns / NANOSECONDS
        tow_tx = Decimal(received_sv_time_ns) / NANOSECONDS
        if constellation == 3:
            tow_tx = _glonass_tow_to_gps(tow_tx, tow_rx)
        elif constellation == 5:
            tow_tx += Decimal(14)
        delta = tow_rx - tow_tx
        while delta > HALF_WEEK:
            delta -= SECONDS_PER_WEEK
        while delta < -HALF_WEEK:
            delta += SECONDS_PER_WEEK
        result = delta * SPEED_OF_LIGHT_MPS
    value = float(result)
    return value if math.isfinite(value) and value > 0.0 else None


def _signal_group(constellation: int, signal: str, frequency_hz: Decimal) -> str | None:
    """Mirror the fixed adapter signal/frequency support for the proxy rows."""
    token = signal.strip().upper()
    frequency = float(frequency_hz)
    tolerance = 1000.0

    def near(value: float) -> bool:
        return abs(frequency - value) <= tolerance

    if constellation == 1:
        if (token in {"GPS_L1_CA", "GPS_L1C", "L1"} or not token) and near(1575420000.0):
            return "GPS_L1CA"
        if (token in {"GPS_L5_Q", "GPS_L5", "L5"} or not token) and near(1176450000.0):
            return "GPS_L5"
    if constellation == 3:
        channel = round((frequency - 1602000000.0) / 562500.0)
        if -7 <= channel <= 6 and near(1602000000.0 + channel * 562500.0):
            if not token or token in {"GLO_G1_CA", "GLO_G1C", "GLO_L1", "L1"}:
                return "GLO_L1CA"
    if constellation == 6:
        if (token in {"GAL_E1_C_P", "GAL_E1_C", "GAL_E1", "L1"} or not token) and near(1575420000.0):
            return "GAL_E1"
        if (token in {"GAL_E5A_Q", "GAL_E5A", "GAL_E5", "L5"} or not token) and near(1176450000.0):
            return "GAL_E5A"
    if constellation == 5:
        if (token in {"BDS_B1I", "BDS_B1_I", "BDS_B1", "B1I"} or not token) and near(1561098000.0):
            return "BDS_B1I"
        if token in {"BDS_B1C", "BDS_B1_C"} and near(1575420000.0):
            return "BDS_B1C"
        if (token in {"BDS_B2A", "BDS_B2A_Q", "BDS_B2A_I", "L5"} or not token) and near(1176450000.0):
            return "BDS_B2A"
    return None


def _constellation_name(constellation: int) -> str:
    return {
        1: "GPS",
        3: "GLONASS",
        5: "BEIDOU",
        6: "GALILEO",
    }.get(constellation, f"CONSTELLATION_{constellation}")


def _median(values: Sequence[float]) -> float | None:
    if not values:
        return None
    value = float(statistics.median(values))
    return value if math.isfinite(value) else None


def _mad(values: Sequence[float], center: float | None = None) -> float | None:
    if not values:
        return None
    midpoint = _median(values) if center is None else center
    if midpoint is None:
        return None
    return _median([abs(value - midpoint) for value in values])


def _percentile(values: Sequence[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    position = (len(ordered) - 1) * percentile / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return float(ordered[lower])
    fraction = position - lower
    return float(ordered[lower] + (ordered[upper] - ordered[lower]) * fraction)


def _distribution(values: Sequence[float]) -> dict[str, Any]:
    finite = [float(value) for value in values if _finite(value)]
    center = _median(finite)
    return {
        "count": len(finite),
        "median": center,
        "mad": _mad(finite, center),
        "p50": _percentile(finite, 50.0),
        "p95_abs": _percentile([abs(value) for value in finite], 95.0),
        "max_abs": max((abs(value) for value in finite), default=None),
        "min": min(finite, default=None),
        "max": max(finite, default=None),
    }


def _group_distribution(values: dict[str, list[float]]) -> dict[str, Any]:
    return {key: _distribution(series) for key, series in sorted(values.items())}


@dataclass
class ProxyRow:
    index: int
    utc_ms: int
    hcdc: int
    svid: int
    constellation: int
    system: str
    signal: str
    frequency_hz: Decimal
    pseudorange_m: float
    full_isb_ns: Decimal | None
    satellite_isb_ns: Decimal | None

    @property
    def group(self) -> str:
        return f"{self.system}:{self.signal}:{self.frequency_hz}Hz"


def _verify_freeze(path: Path = FREEZE) -> dict[str, Any]:
    freeze, digest = _load_json_once(path, "Phase60 freeze", FREEZE_SHA256)
    if digest != FREEZE_SHA256:
        raise _fail(f"Phase60 freeze hash changed: {digest} != {FREEZE_SHA256}")
    if freeze.get("schema_version") != FREEZE_SCHEMA or freeze.get("status") != "frozen-before-phase60-raw-read":
        raise _fail("Phase60 freeze schema/status mismatch")
    objective = freeze.get("objective", {})
    if objective.get("truth_free") is not True or objective.get("solver_rerun") is not False:
        raise _fail("Phase60 objective permits forbidden input")
    cohort = freeze.get("cohort", {})
    if tuple(cohort.get("route_order", [])) != ROUTES or cohort.get("route_disjoint") is not True:
        raise _fail("Phase60 cohort route order changed")
    raw_inputs = freeze.get("exact_raw_inputs", {})
    if tuple(raw_inputs) != ROUTES:
        raise _fail("Phase60 raw input route map changed")
    for route in ROUTES:
        pin = raw_inputs[route]
        expected = ROUTE_INPUTS[route]
        if pin.get("path") != expected["path"] or pin.get("bytes") != expected["bytes"] or pin.get("sha256") != expected["sha256"]:
            raise _fail(f"Phase60 raw input pin changed: {route}")
    semantics = freeze.get("source_semantics", {})
    full = semantics.get("full_inter_signal_bias", {})
    satellite = semantics.get("satellite_inter_signal_bias", {})
    if full.get("units") != "nanoseconds" or satellite.get("units") != "nanoseconds":
        raise _fail("Phase60 ISB units changed")
    if full.get("sign_equation") != "corrected pseudorange = raw pseudorange - FullInterSignalBiasNanos":
        raise _fail("Phase60 Full ISB sign changed")
    if satellite.get("sign_equation") != "corrected pseudorange = raw pseudorange - SatelliteInterSignalBiasNanos":
        raise _fail("Phase60 Satellite ISB sign changed")
    if "Full plus Satellite" not in semantics.get("composition_rule", ""):
        raise _fail("Phase60 double-count prohibition missing")
    if "FullISB_a" not in semantics.get("pair_full_corrected_delta_m", ""):
        raise _fail("Phase60 pair formula changed")
    source_contracts = freeze.get("authority_pins", {}).get("source_contracts", {})
    if not isinstance(source_contracts, dict) or len(source_contracts) < 7:
        raise _fail("Phase60 source contract pins incomplete")
    gates = freeze.get("numeric_gates", {})
    if gates.get("route_count") != 4:
        raise _fail("Phase60 route gate changed")
    if gates.get("adopted_proxy_coverage", {}).get("field_header_presence_required_for_authorization") is not True:
        raise _fail("Phase60 field-presence gate changed")
    if gates.get("multi_signal_pair", {}).get("same_epoch_only") is not True or gates.get("multi_signal_pair", {}).get("same_satellite_only") is not True:
        raise _fail("Phase60 pair eligibility changed")
    if gates.get("bias_materiality", {}).get("not_pure_common_mode") is not True:
        raise _fail("Phase60 common-mode gate changed")
    policy = freeze.get("input_policy", {})
    if policy.get("single_evaluator_process") is not True or policy.get("raw_device_gnss_read_count_per_route") != 1 or policy.get("raw_device_gnss_read_count_total") != 4:
        raise _fail("Phase60 raw read policy changed")
    if any(policy.get(name) not in (0, False) for name in (
        "raw_device_imu_read_count", "truth_read_count", "prior_metric_payload_reads",
        "brdc_nav_read_count", "solver_or_trajectory_reruns", "coordinate_or_wls_inputs",
        "mat_reads_or_generated", "validation_holdout_reads", "archive_reopens",
        "rematerialization_count", "kaggle_or_token_access", "correction_fit_or_application_before_gate",
    )):
        raise _fail("Phase60 forbidden input policy changed")
    assertions = freeze.get("pre_read_assertions", {})
    if not isinstance(assertions, dict) or any(value is not False for value in assertions.values()):
        raise _fail("Phase60 pre-read assertions are not closed")
    return freeze


def _verify_manifest(freeze: dict[str, Any], path: Path = MANIFEST) -> dict[str, Any]:
    global VERIFIED_MANIFEST_SHA256
    manifest, digest = _load_json_once(path, "Phase60 evaluator manifest", MANIFEST_SHA256 or None)
    if MANIFEST_SHA256 and digest != MANIFEST_SHA256:
        raise _fail(f"Phase60 evaluator manifest hash changed: {digest} != {MANIFEST_SHA256}")
    if manifest.get("schema_version") != MANIFEST_SCHEMA or manifest.get("status") != "evaluator-frozen-before-phase60-raw-read":
        raise _fail("Phase60 evaluator manifest schema/status mismatch")
    freeze_pin = manifest.get("freeze", {})
    if freeze_pin.get("path") != _relative(FREEZE) or freeze_pin.get("sha256") != FREEZE_SHA256:
        raise _fail("Phase60 evaluator manifest freeze pin mismatch")
    evaluator = manifest.get("evaluator", {})
    if evaluator.get("single_process") is not True or evaluator.get("raw_reads_per_route") != 1 or evaluator.get("truth_reads") != 0 or evaluator.get("solver_invocations") != 0:
        raise _fail("Phase60 evaluator manifest permits forbidden input")
    forbidden = manifest.get("forbidden", [])
    required_forbidden = (
        "truth", "navigation", "solver", "trajectory", "coordinates", "device WLS",
        "SvPosition", "SvElevation", "IMU", "MAT", "validation", "holdout",
        "archive", "Kaggle", "token", "prior metric payload", "enriched RawPseudorangeMeters",
    )
    if not isinstance(forbidden, list) or not all(item in forbidden for item in required_forbidden):
        raise _fail("Phase60 forbidden input manifest incomplete")
    for name in ("source", "test", "cmake"):
        pin = evaluator.get(name, {})
        pin_path = ROOT / str(pin.get("path", ""))
        if not pin_path.is_file() or len(str(pin.get("sha256", ""))) != 64:
            raise _fail(f"Phase60 {name} pin missing")
        payload = pin_path.read_bytes()
        if _sha256_bytes(payload) != pin["sha256"]:
            raise _fail(f"Phase60 {name} pin hash mismatch")
    VERIFIED_MANIFEST_SHA256 = digest
    return manifest


def _source_audit(freeze: dict[str, Any]) -> dict[str, Any]:
    contents: dict[str, str] = {}
    hashes: dict[str, str] = {}
    for name, pin in freeze["authority_pins"]["source_contracts"].items():
        path = ROOT / str(pin["path"])
        payload, digest = _read_bytes_once(path, f"Phase60 source {name}", str(pin["sha256"]))
        try:
            contents[name] = payload.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise _fail(f"Phase60 source is not UTF-8: {name}") from exc
        hashes[name] = digest
    adapter = contents["android_raw_gnss_cpp"].lower()
    adapter_header = contents["android_raw_gnss_header"].lower()
    observation = contents["observation_header"].lower()
    fgo = contents["fgo_problems_cpp"].lower()
    fgo_header = contents["fgo_header"].lower()
    cli = contents["native_fgo_cli"].lower()
    return {
        "source_hashes": hashes,
        "adapter": {
            "parses_full_intersignal_bias": "fullintersignalbiasnanos" in adapter and "full_inter_signal_bias" in adapter,
            "parses_satellite_intersignal_bias": "satelliteintersignalbiasnanos" in adapter and "satellite_inter_signal_bias" in adapter,
            "header_declares_full_intersignal_bias": "fullintersignalbias" in adapter_header,
            "header_declares_satellite_intersignal_bias": "satelliteintersignalbias" in adapter_header,
            "parses_raw_pseudorange_clock": "receivedsvtimenanos" in adapter and "fullbiasnanos" in adapter,
        },
        "observation": {
            "retains_full_intersignal_bias": "fullintersignalbias" in observation,
            "retains_satellite_intersignal_bias": "satelliteintersignalbias" in observation,
            "retains_generic_antenna_pco_only": "antenna_pco" in observation,
        },
        "fgo": {
            "consumes_raw_full_intersignal_bias": "fullintersignalbias" in fgo,
            "consumes_raw_satellite_intersignal_bias": "satelliteintersignalbias" in fgo,
            "existing_signal_bias_state_path_present": "signal_bias" in fgo or "signalbias" in fgo_header,
            "adopted_pseudorange_factor_path_present": "corrected_pseudorange" in fgo and "pseudorangefactor" in fgo,
        },
        "cli": {
            "raw_full_intersignal_option": "fullintersignalbias" in cli,
            "raw_satellite_intersignal_option": "satelliteintersignalbias" in cli,
        },
        "interpretation": "Existing signal-bias states/factors are estimator state machinery and are not evidence that either Android raw ISB field is parsed, retained, or consumed.",
    }


def _header_map(fieldnames: Sequence[str]) -> dict[str, str]:
    result: dict[str, str] = {}
    for field in fieldnames:
        key = _normalise_header(field)
        if not key or key in result:
            raise _fail("Phase60 raw header has an empty or duplicate normalized field")
        result[key] = field
    return result


def _field_value(row: dict[str, str], columns: dict[str, str], name: str) -> str:
    column = columns.get(_normalise_header(name))
    return "" if column is None else row.get(column, "")


def _parse_raw_route(route: str, freeze: dict[str, Any]) -> dict[str, Any]:
    pin = freeze["exact_raw_inputs"][route]
    path = ROOT / str(pin["path"])
    payload, digest = _read_bytes_once(
        path,
        f"Phase60 raw {route}",
        str(pin["sha256"]),
        int(pin["bytes"]),
    )
    try:
        text = payload.decode("utf-8-sig")
    except UnicodeDecodeError as exc:
        raise _fail(f"Phase60 raw input is not UTF-8: {route}") from exc
    stream = io.StringIO(text, newline="")
    reader = __import__("csv").DictReader(stream)
    fieldnames = reader.fieldnames or []
    columns = _header_map(fieldnames)
    missing = [name for name in REQUIRED_COLUMNS if _normalise_header(name) not in columns]
    if missing:
        raise _fail(f"Phase60 raw header lacks required fields for {route}: {missing}")
    header_presence = {name: _normalise_header(name) in columns for name in OPTIONAL_COLUMNS}
    raw_rows = 0
    non_raw_rows = 0
    malformed_core_rows = 0
    malformed_optional_rows = 0
    unsupported_rows = 0
    invalid_timing_rows = 0
    invalid_quality_rows = 0
    low_snr_rows = 0
    multipath_rows = 0
    code_masked_rows = 0
    adopted: list[ProxyRow] = []
    epoch_order: list[int] = []
    epoch_seen: set[int] = set()
    nonmonotonic = 0
    duplicate_epochs = 0
    previous_utc: int | None = None
    previous_time_ns: int | None = None
    previous_hcdc: int | None = None
    base_full_bias_ns: int | None = None

    for source_index, source_row in enumerate(reader, start=2):
        message_type = _field_value(source_row, columns, "MessageType").strip()
        if message_type != "Raw":
            non_raw_rows += 1
            continue
        raw_rows += 1
        try:
            utc_ms = _parse_int(_field_value(source_row, columns, "utcTimeMillis"))
            time_ns = _parse_int(_field_value(source_row, columns, "TimeNanos"))
            full_bias_ns = _parse_int(_field_value(source_row, columns, "FullBiasNanos"))
            received_sv_time_ns = _parse_int(_field_value(source_row, columns, "ReceivedSvTimeNanos"))
            bias_ns = _parse_decimal(_field_value(source_row, columns, "BiasNanos"))
            frequency_hz = _parse_decimal(_field_value(source_row, columns, "CarrierFrequencyHz"))
            _parse_decimal(_field_value(source_row, columns, "PseudorangeRateMetersPerSecond"))
            _parse_int(_field_value(source_row, columns, "AccumulatedDeltaRangeState"))
            svid = _parse_int(_field_value(source_row, columns, "Svid"))
            constellation = _parse_int(_field_value(source_row, columns, "ConstellationType"))
            if any(value is None for value in (utc_ms, time_ns, full_bias_ns, received_sv_time_ns, bias_ns, frequency_hz, svid, constellation)):
                raise ValueError("missing core value")
            time_offset_ns = _parse_decimal(_field_value(source_row, columns, "TimeOffsetNanos"), required=False) or Decimal(0)
            hcdc = _parse_int(_field_value(source_row, columns, "HardwareClockDiscontinuityCount"), required=False) or 0
            state = _parse_int(_field_value(source_row, columns, "State"), required=False)
            multipath = _parse_int(_field_value(source_row, columns, "MultipathIndicator"), required=False)
            cn0 = _parse_decimal(_field_value(source_row, columns, "Cn0DbHz"), required=False)
            full_isb_ns = _parse_decimal(_field_value(source_row, columns, "FullInterSignalBiasNanos"), required=False)
            satellite_isb_ns = _parse_decimal(_field_value(source_row, columns, "SatelliteInterSignalBiasNanos"), required=False)
            optional_tokens = (
                "TimeOffsetNanos", "HardwareClockDiscontinuityCount", "State",
                "MultipathIndicator", "Cn0DbHz", "FullInterSignalBiasNanos",
                "SatelliteInterSignalBiasNanos", "FullInterSignalBiasUncertaintyNanos",
                "SatelliteInterSignalBiasUncertaintyNanos",
            )
            for optional in optional_tokens:
                token = _field_value(source_row, columns, optional).strip()
                if token and _parse_decimal(token, required=False) is None and optional not in {"HardwareClockDiscontinuityCount", "State", "MultipathIndicator"}:
                    # NaN/Inf is an unavailable optional value, not malformed.
                    try:
                        parsed_optional = Decimal(token)
                        if parsed_optional.is_finite():
                            malformed_optional_rows += 1
                    except InvalidOperation:
                        malformed_optional_rows += 1
        except (InvalidOperation, ValueError, TypeError, OverflowError):
            malformed_core_rows += 1
            continue
        if previous_utc is not None and utc_ms < previous_utc:
            nonmonotonic += 1
        if previous_time_ns is not None and abs(time_ns - previous_time_ns) > TIME_GAP_NS:
            base_full_bias_ns = full_bias_ns
        if base_full_bias_ns is None:
            base_full_bias_ns = full_bias_ns
        if previous_hcdc is None:
            previous_hcdc = hcdc
        if previous_utc != utc_ms:
            if utc_ms in epoch_seen:
                duplicate_epochs += 1
            epoch_seen.add(utc_ms)
            epoch_order.append(utc_ms)
        previous_utc = utc_ms
        previous_time_ns = time_ns
        previous_hcdc = hcdc
        if time_ns == 0 or received_sv_time_ns < 10_000_000_000:
            invalid_timing_rows += 1
            continue
        system_signal = _signal_group(constellation, _field_value(source_row, columns, "SignalType"), frequency_hz)
        if system_signal is None:
            unsupported_rows += 1
            continue
        pseudorange_m = _raw_pseudorange_m(
            time_ns, base_full_bias_ns, bias_ns, time_offset_ns,
            received_sv_time_ns, constellation,
        )
        if pseudorange_m is None:
            invalid_quality_rows += 1
            continue
        low_snr = cn0 is not None and cn0 < Decimal(20)
        multipath_flag = multipath == 1
        if low_snr:
            low_snr_rows += 1
        if multipath_flag:
            multipath_rows += 1
        code_lock_mask = 1 | (1 << 10)
        transmit_mask = (1 << 7) | (1 << 15) if constellation == 3 else (1 << 3) | (1 << 14)
        state_invalid = state is not None and ((state & code_lock_mask) == 0 or (state & transmit_mask) == 0)
        code_masked = low_snr or multipath_flag or pseudorange_m < float(MIN_P_M) or pseudorange_m > float(MAX_P_M) or state_invalid
        if code_masked:
            code_masked_rows += 1
            continue
        system = _constellation_name(constellation)
        row = ProxyRow(
            index=source_index,
            utc_ms=utc_ms,
            hcdc=hcdc,
            svid=svid,
            constellation=constellation,
            system=system,
            signal=system_signal,
            frequency_hz=frequency_hz,
            pseudorange_m=pseudorange_m,
            full_isb_ns=full_isb_ns,
            satellite_isb_ns=satellite_isb_ns,
        )
        adopted.append(row)

    if not epoch_order and raw_rows:
        raise _fail(f"Phase60 raw input has no valid epoch keys: {route}")
    field_values: dict[str, list[float]] = {"full": [], "satellite": [], "receiver_remainder": []}
    field_by_group: dict[str, dict[str, list[float]]] = {
        "full": defaultdict(list),
        "satellite": defaultdict(list),
        "receiver_remainder": defaultdict(list),
    }
    finite_by_field = {"full": 0, "satellite": 0, "receiver_remainder": 0}
    common_groups: dict[str, dict[tuple[int, int], list[float]]] = {
        "full": defaultdict(list), "satellite": defaultdict(list), "receiver_remainder": defaultdict(list)
    }
    pair_groups: dict[tuple[int, int, int], list[ProxyRow]] = defaultdict(list)
    temporal_groups: dict[str, list[tuple[int, float]]] = defaultdict(list)
    signal_counts: dict[str, int] = defaultdict(int)
    satellite_counts: dict[str, int] = defaultdict(int)
    for row in adopted:
        signal_counts[row.group] += 1
        satellite_counts[f"{row.system}:SVID{row.svid}"] += 1
        pair_groups[(row.utc_ms, row.constellation, row.svid)].append(row)
        for field, value_ns in (("full", row.full_isb_ns), ("satellite", row.satellite_isb_ns)):
            value_m = _bias_m(value_ns)
            if value_m is not None:
                finite_by_field[field] += 1
                field_values[field].append(value_m)
                field_by_group[field][row.group].append(value_m)
                common_groups[field][(row.utc_ms, row.hcdc)].append(value_m)
                temporal_groups[f"{field}:{row.system}:{row.svid}:{row.signal}"].append((row.utc_ms, value_m))
        full_m = _bias_m(row.full_isb_ns)
        satellite_m = _bias_m(row.satellite_isb_ns)
        if full_m is not None and satellite_m is not None:
            remainder = full_m - satellite_m
            finite_by_field["receiver_remainder"] += 1
            field_values["receiver_remainder"].append(remainder)
            field_by_group["receiver_remainder"][row.group].append(remainder)
            common_groups["receiver_remainder"][(row.utc_ms, row.hcdc)].append(remainder)

    pair_count = 0
    pair_finite_full = 0
    pair_finite_satellite = 0
    pair_finite_both = 0
    pair_bias_diff_full: list[float] = []
    pair_bias_diff_satellite: list[float] = []
    pair_corrected_shift_full: list[float] = []
    pair_corrected_shift_satellite: list[float] = []
    pair_full_corrected_delta: list[float] = []
    pair_satellite_corrected_delta: list[float] = []
    pair_raw_delta: list[float] = []
    pair_group_counts: dict[str, int] = defaultdict(int)
    pair_satellite_ids: set[str] = set()
    for grouped in pair_groups.values():
        distinct = {}
        for row in grouped:
            distinct.setdefault(row.group, row)
        if len(distinct) < 2:
            continue
        ordered = sorted(distinct.values(), key=lambda item: (item.group, item.index))
        for first, second in itertools.combinations(ordered, 2):
            pair_count += 1
            pair_satellite_ids.add(f"{first.system}:SVID{first.svid}")
            pair_group_counts[f"{first.group}__{second.group}"] += 1
            raw_delta = first.pseudorange_m - second.pseudorange_m
            pair_raw_delta.append(raw_delta)
            full_a = _bias_m(first.full_isb_ns)
            full_b = _bias_m(second.full_isb_ns)
            sat_a = _bias_m(first.satellite_isb_ns)
            sat_b = _bias_m(second.satellite_isb_ns)
            if full_a is not None and full_b is not None:
                pair_finite_full += 1
                difference = full_a - full_b
                pair_bias_diff_full.append(difference)
                pair_corrected_shift_full.append(-difference)
                pair_full_corrected_delta.append(raw_delta - difference)
            if sat_a is not None and sat_b is not None:
                pair_finite_satellite += 1
                difference = sat_a - sat_b
                pair_bias_diff_satellite.append(difference)
                pair_corrected_shift_satellite.append(-difference)
                pair_satellite_corrected_delta.append(raw_delta - difference)
            if full_a is not None and full_b is not None and sat_a is not None and sat_b is not None:
                pair_finite_both += 1

    common_mode: dict[str, Any] = {}
    for field in ("full", "satellite", "receiver_remainder"):
        centers: list[float] = []
        centered: list[float] = []
        for values in common_groups[field].values():
            center = _median(values)
            if center is None:
                continue
            centers.append(center)
            centered.extend(value - center for value in values)
        common_mode[field] = {
            "epoch_group_count": len(common_groups[field]),
            "common_epoch_median": _median(centers),
            "common_epoch_median_distribution": _distribution(centers),
            "noncommon_centered_distribution": _distribution(centered),
        }

    temporal: dict[str, Any] = {}
    event_rows: list[dict[str, Any]] = []
    for field in ("full", "satellite", "receiver_remainder"):
        changes: list[float] = []
        total_adjacent = 0
        jumps = 0
        for key, values in temporal_groups.items():
            if not key.startswith(field + ":"):
                continue
            values.sort(key=lambda item: item[0])
            for previous, current in zip(values, values[1:]):
                total_adjacent += 1
                change = current[1] - previous[1]
                changes.append(change)
                if abs(change) > BIAS_JUMP_M:
                    jumps += 1
                    if len(event_rows) < 10000:
                        event_rows.append({
                            "route": route,
                            "field": field,
                            "series": key,
                            "utcTimeMillis_previous": previous[0],
                            "utcTimeMillis_current": current[0],
                            "change_m": change,
                        })
        temporal[field] = {
            "adjacent_count": total_adjacent,
            "change_distribution_m": _distribution(changes),
            "finite_adjacent_fraction": (total_adjacent / max(finite_by_field[field], 1)) if finite_by_field[field] else 0.0,
            "jump_threshold_m": BIAS_JUMP_M,
            "jump_count": jumps,
            "jump_fraction": (jumps / total_adjacent) if total_adjacent else 0.0,
        }

    field_availability: dict[str, Any] = {}
    for field_name, header_name in (("full", "FullInterSignalBiasNanos"), ("satellite", "SatelliteInterSignalBiasNanos")):
        values = field_values[field_name]
        field_availability[field_name] = {
            "header_present": header_presence[header_name],
            "adopted_proxy_rows": len(adopted),
            "finite_rows": finite_by_field[field_name],
            "finite_fraction": finite_by_field[field_name] / len(adopted) if adopted else 0.0,
            "distribution_m": _distribution(values),
            "by_signal_frequency_group": _group_distribution(field_by_group[field_name]),
        }
    field_availability["receiver_remainder"] = {
        "finite_rows": finite_by_field["receiver_remainder"],
        "finite_fraction_of_adopted": finite_by_field["receiver_remainder"] / len(adopted) if adopted else 0.0,
        "distribution_m": _distribution(field_values["receiver_remainder"]),
        "by_signal_frequency_group": _group_distribution(field_by_group["receiver_remainder"]),
    }

    pair_report = {
        "all_same_epoch_same_satellite_pairs": pair_count,
        "finite_full_pairs": pair_finite_full,
        "finite_satellite_pairs": pair_finite_satellite,
        "finite_both_pairs": pair_finite_both,
        "distinct_satellites_with_pairs": len(pair_satellite_ids),
        "signal_frequency_pair_counts": dict(sorted(pair_group_counts.items())),
        "raw_pair_delta_m": _distribution(pair_raw_delta),
        "full_bias_difference_m": _distribution(pair_bias_diff_full),
        "satellite_bias_difference_m": _distribution(pair_bias_diff_satellite),
        "full_correction_shift_m": _distribution(pair_corrected_shift_full),
        "satellite_correction_shift_m": _distribution(pair_corrected_shift_satellite),
        "full_corrected_pair_delta_m": _distribution(pair_full_corrected_delta),
        "satellite_corrected_pair_delta_m": _distribution(pair_satellite_corrected_delta),
        "source_sign": "corrected=raw-bias; shifts are -(bias_a-bias_b)",
        "no_full_plus_satellite": True,
    }
    common_signal_groups = len(signal_counts)
    distinct_satellites = len(satellite_counts)
    raw_integrity = {
        "exact_sha256": digest == str(pin["sha256"]),
        "exact_bytes": len(payload) == int(pin["bytes"]),
        "core_raw_fields_finite": malformed_core_rows == 0,
        "malformed_core_rows": malformed_core_rows,
        "malformed_optional_rows": malformed_optional_rows,
        "utc_epoch_nonmonotonic_count": nonmonotonic,
        "duplicate_epoch_key_count": duplicate_epochs,
        "unsupported_signal_rows_informational": unsupported_rows,
        "all_core_integrity": malformed_core_rows == 0 and nonmonotonic == 0 and duplicate_epochs == 0,
    }
    return {
        "route": route,
        "raw_input": {
            "path": str(pin["path"]),
            "bytes": len(payload),
            "sha256": digest,
        },
        "header": {
            "columns": list(fieldnames),
            "required_columns_present": True,
            "optional_field_presence": header_presence,
        },
        "raw_rows": raw_rows,
        "non_raw_rows": non_raw_rows,
        "epoch_count": len(epoch_order),
        "adopted_proxy_rows": len(adopted),
        "distinct_satellites": distinct_satellites,
        "signal_frequency_group_count": common_signal_groups,
        "signal_frequency_group_counts": dict(sorted(signal_counts.items())),
        "unsupported_signal_rows": unsupported_rows,
        "invalid_timing_rows": invalid_timing_rows,
        "invalid_quality_rows": invalid_quality_rows,
        "low_snr_rows": low_snr_rows,
        "multipath_rows": multipath_rows,
        "code_masked_rows": code_masked_rows,
        "raw_input_integrity": raw_integrity,
        "field_availability": field_availability,
        "pair_diagnostics": pair_report,
        "common_vs_noncommon": common_mode,
        "temporal_stability": temporal,
        "route_events": event_rows,
        "_aggregate_values": {
            "full": field_values["full"],
            "satellite": field_values["satellite"],
            "receiver_remainder": field_values["receiver_remainder"],
            "pair_full_difference": pair_bias_diff_full,
            "pair_satellite_difference": pair_bias_diff_satellite,
            "pair_full_shift": pair_corrected_shift_full,
            "pair_satellite_shift": pair_corrected_shift_satellite,
            "pair_raw_delta": pair_raw_delta,
        },
    }


def _aggregate_route_values(route_reports: Sequence[dict[str, Any]], field: str) -> list[float]:
    values: list[float] = []
    for report in route_reports:
        values.extend(report.get("_aggregate_values", {}).get(field, []))
    return values


def _loo_summary(route_reports: Sequence[dict[str, Any]]) -> dict[str, Any]:
    folds: dict[str, Any] = {}
    for held_out in ROUTES:
        remaining = [report for report in route_reports if report["route"] != held_out]
        full = _aggregate_route_values(remaining, "full")
        satellite = _aggregate_route_values(remaining, "satellite")
        remainder = _aggregate_route_values(remaining, "receiver_remainder")
        folds[held_out] = {
            "held_out_route": held_out,
            "remaining_route_count": len(remaining),
            "full_bias_distribution_m": _distribution(full),
            "satellite_bias_distribution_m": _distribution(satellite),
            "receiver_remainder_distribution_m": _distribution(remainder),
            "full_header_or_finite_present": bool(full),
            "satellite_header_or_finite_present": bool(satellite),
        }
    return {"fold_count": len(folds), "folds": folds}


def _route_gates(report: dict[str, Any], freeze: dict[str, Any]) -> dict[str, bool]:
    gates = freeze["numeric_gates"]
    integrity = report["raw_input_integrity"]
    coverage = gates["adopted_proxy_coverage"]
    pairs = gates["multi_signal_pair"]
    materiality = gates["bias_materiality"]
    availability = report["field_availability"]
    pair_report = report["pair_diagnostics"]
    temporal = report["temporal_stability"]
    full_dist = availability["full"]["distribution_m"]
    sat_dist = availability["satellite"]["distribution_m"]
    remainder_dist = availability["receiver_remainder"]["distribution_m"]
    full_pair_dist = pair_report["full_bias_difference_m"]
    sat_pair_dist = pair_report["satellite_bias_difference_m"]
    raw_shift = pair_report["full_correction_shift_m"]
    return {
        "raw_input_integrity": integrity["all_core_integrity"],
        "adopted_rows": report["adopted_proxy_rows"] >= coverage["min_rows_per_route"],
        "satellites": report["distinct_satellites"] >= coverage["min_distinct_satellites_per_route"],
        "full_header_present": availability["full"]["header_present"],
        "satellite_header_present": availability["satellite"]["header_present"],
        "full_finite_coverage": availability["full"]["finite_fraction"] >= coverage["full_field_finite_fraction_min"],
        "satellite_finite_coverage": availability["satellite"]["finite_fraction"] >= coverage["satellite_field_finite_fraction_min"],
        "same_epoch_same_satellite_pairs": pair_report["all_same_epoch_same_satellite_pairs"] >= pairs["min_pairs_per_route"],
        "pair_satellites": pair_report["distinct_satellites_with_pairs"] >= pairs["min_distinct_satellites_with_pairs_per_route"],
        "signal_frequency_groups": report["signal_frequency_group_count"] >= pairs["min_distinct_signal_frequency_groups_per_route"],
        "signal_families": len({key.split(":", 2)[1] for key in report["signal_frequency_group_counts"]}) >= pairs["min_signal_families_per_route"],
        "signed_full_materiality": (full_dist["p95_abs"] or 0.0) >= materiality["signed_field_p95_abs_min_m"],
        "signed_satellite_materiality": (sat_dist["p95_abs"] or 0.0) >= materiality["signed_field_p95_abs_min_m"],
        "receiver_remainder_materiality": (remainder_dist["p95_abs"] or 0.0) >= materiality["full_minus_satellite_receiver_remainder_p95_abs_min_m"],
        "pair_full_materiality": (full_pair_dist["p95_abs"] or 0.0) >= materiality["pair_bias_difference_p95_abs_min_m"],
        "pair_satellite_materiality": (sat_pair_dist["p95_abs"] or 0.0) >= materiality["pair_bias_difference_p95_abs_min_m"],
        "pair_full_shift_materiality": (raw_shift["p95_abs"] or 0.0) >= materiality["pair_corrected_vs_raw_p95_excess_min_m"],
        "full_temporal_coverage": temporal["full"]["finite_adjacent_fraction"] >= gates["temporal_and_loo"]["finite_adjacent_field_fraction_min"],
        "satellite_temporal_coverage": temporal["satellite"]["finite_adjacent_fraction"] >= gates["temporal_and_loo"]["finite_adjacent_field_fraction_min"],
        "jump_fraction": temporal["full"]["jump_fraction"] <= gates["temporal_and_loo"]["max_unexplained_jump_fraction"] and temporal["satellite"]["jump_fraction"] <= gates["temporal_and_loo"]["max_unexplained_jump_fraction"],
        "full_satellite_decomposition": pair_report["no_full_plus_satellite"],
        "all_reported_finite": all(
            value is None or _finite(value)
            for value in (
                full_dist.get("median"), full_dist.get("p95_abs"), sat_dist.get("median"), sat_dist.get("p95_abs"),
                remainder_dist.get("median"), remainder_dist.get("p95_abs"), full_pair_dist.get("median"), sat_pair_dist.get("median"),
            )
        ),
    }


def _audit(freeze: dict[str, Any], manifest: dict[str, Any], output_root: Path) -> dict[str, Any]:
    _reject_path(output_root)
    if output_root.exists():
        raise _fail(f"Phase60 output already exists: {output_root}")
    source = _source_audit(freeze)
    route_reports: list[dict[str, Any]] = []
    read_count = 0
    for route in ROUTES:
        report = _parse_raw_route(route, freeze)
        read_count += 1
        report["gates"] = _route_gates(report, freeze)
        route_reports.append(report)

    route_gate_names = sorted(route_reports[0]["gates"]) if route_reports else []
    aggregate_route_gates = {
        name: all(report["gates"].get(name, False) for report in route_reports)
        for name in route_gate_names
    }
    raw_integrity_all = all(report["raw_input_integrity"]["all_core_integrity"] for report in route_reports)
    full_all = _aggregate_route_values(route_reports, "full")
    satellite_all = _aggregate_route_values(route_reports, "satellite")
    remainder_all = _aggregate_route_values(route_reports, "receiver_remainder")
    pair_full_all = _aggregate_route_values(route_reports, "pair_full_difference")
    pair_satellite_all = _aggregate_route_values(route_reports, "pair_satellite_difference")
    aggregate = {
        "full_bias_distribution_m": _distribution(full_all),
        "satellite_bias_distribution_m": _distribution(satellite_all),
        "receiver_remainder_distribution_m": _distribution(remainder_all),
        "pair_full_bias_difference_m": _distribution(pair_full_all),
        "pair_satellite_bias_difference_m": _distribution(pair_satellite_all),
        "route_medians": {
            "full": [report["field_availability"]["full"]["distribution_m"]["median"] for report in route_reports],
            "satellite": [report["field_availability"]["satellite"]["distribution_m"]["median"] for report in route_reports],
            "receiver_remainder": [report["field_availability"]["receiver_remainder"]["distribution_m"]["median"] for report in route_reports],
        },
        "route_order": list(ROUTES),
    }
    loo = _loo_summary(route_reports)
    source_checks = {
        "adapter_does_not_parse_full_intersignal_bias": source["adapter"]["parses_full_intersignal_bias"] is False,
        "adapter_does_not_parse_satellite_intersignal_bias": source["adapter"]["parses_satellite_intersignal_bias"] is False,
        "observation_does_not_retain_full_intersignal_bias": source["observation"]["retains_full_intersignal_bias"] is False,
        "observation_does_not_retain_satellite_intersignal_bias": source["observation"]["retains_satellite_intersignal_bias"] is False,
        "fgo_does_not_consume_full_intersignal_bias": source["fgo"]["consumes_raw_full_intersignal_bias"] is False,
        "fgo_does_not_consume_satellite_intersignal_bias": source["fgo"]["consumes_raw_satellite_intersignal_bias"] is False,
        "existing_signal_bias_state_not_raw_field": source["fgo"]["existing_signal_bias_state_path_present"],
    }
    presentation = {
        "route_count_exact": len(route_reports) == 4,
        "route_order_exact": [report["route"] for report in route_reports] == list(ROUTES),
        "route_row_counts_sum": sum(report["adopted_proxy_rows"] for report in route_reports) >= 0,
        "field_availability_counts_sum": all(
            report["field_availability"][field]["finite_rows"] <= report["adopted_proxy_rows"]
            for report in route_reports for field in ("full", "satellite")
        ),
        "pair_counts_sum": all(
            report["pair_diagnostics"]["finite_both_pairs"] <= report["pair_diagnostics"]["all_same_epoch_same_satellite_pairs"]
            for report in route_reports
        ),
        "four_route_medians_retained": len(aggregate["route_medians"]["full"]) == 4 and len(aggregate["route_medians"]["satellite"]) == 4,
        "loo_fold_count_exact": loo["fold_count"] == 4,
        "read_count_exact": read_count == 4,
    }
    gates = {
        "route_count": len(route_reports) == 4,
        "raw_input_integrity": raw_integrity_all,
        "adopted_proxy_coverage": aggregate_route_gates.get("adopted_rows", False) and aggregate_route_gates.get("satellites", False),
        "field_headers": aggregate_route_gates.get("full_header_present", False) and aggregate_route_gates.get("satellite_header_present", False),
        "field_finite_coverage": aggregate_route_gates.get("full_finite_coverage", False) and aggregate_route_gates.get("satellite_finite_coverage", False),
        "same_satellite_multi_signal_pairs": aggregate_route_gates.get("same_epoch_same_satellite_pairs", False) and aggregate_route_gates.get("pair_satellites", False),
        "signal_composition": aggregate_route_gates.get("signal_frequency_groups", False) and aggregate_route_gates.get("signal_families", False),
        "bias_materiality": all(aggregate_route_gates.get(name, False) for name in (
            "signed_full_materiality", "signed_satellite_materiality", "receiver_remainder_materiality",
            "pair_full_materiality", "pair_satellite_materiality", "pair_full_shift_materiality",
        )),
        "temporal_stability": all(aggregate_route_gates.get(name, False) for name in ("full_temporal_coverage", "satellite_temporal_coverage", "jump_fraction")),
        "source_sign_and_decomposition": all(source_checks.values()),
        "presentation_integrity": all(presentation.values()),
        "loo_route_count": loo["fold_count"] == freeze["numeric_gates"]["temporal_and_loo"]["loo_route_count_exact"],
    }
    all_passed = all(gates.values())
    if all_passed:
        status = "phase60-go-intersignal-bias-identifiable-audit-only"
        decision = {
            "native_correction_authorized": False,
            "implementation_stage_authorization": "separately freeze before source edits; FGO undifferenced pseudorange only",
            "candidate_formula": "P_corrected=P_raw-c*FullInterSignalBiasNanos*1e-9; SatelliteInterSignalBiasNanos retained as provenance; never add",
            "next_single_factor": None,
        }
    else:
        status = "phase60-no-go-intersignal-bias-not-identifiable"
        failed = [name for name, value in gates.items() if not value]
        decision = {
            "native_correction_authorized": False,
            "implementation_stage_authorization": False,
            "failed_gates": failed,
            "strongest_finding": "FullInterSignalBiasNanos/SatelliteInterSignalBiasNanos lack the required finite, route-stable, same-satellite multi-signal non-common evidence under the frozen source-sign contract.",
            "next_single_factor": freeze["failure_policy"]["next_single_factor_if_no_go"],
        }
    reads = {
        "single_process": True,
        "raw_device_gnss_reads_per_route": 1,
        "raw_device_gnss_reads_total": read_count,
        "truth_reads": 0,
        "prior_metric_payload_reads": 0,
        "brdc_nav_reads": 0,
        "solver_invocations": 0,
        "trajectory_reruns": 0,
        "raw_device_imu_reads": 0,
        "coordinate_or_wls_inputs": 0,
        "SvPosition_or_SvElevation": 0,
        "enriched_RawPseudorange_inputs": 0,
        "mat_reads_or_generated": 0,
        "validation_holdout_reads": 0,
        "archive_reopens": 0,
        "rematerialization_count": 0,
        "kaggle_or_token_access": 0,
        "correction_fit_or_application": 0,
    }
    public_routes: list[dict[str, Any]] = []
    all_events: list[dict[str, Any]] = []
    for report in route_reports:
        report_copy = {key: value for key, value in report.items() if key not in {"_aggregate_values", "route_events"}}
        public_routes.append(report_copy)
        all_events.extend(report["route_events"])
    result = {
        "schema_version": SCHEMA,
        "phase": 60,
        "execution_label": "Luna Max",
        "status": status,
        "operation": "truth-free raw Android FullInterSignalBiasNanos + SatelliteInterSignalBiasNanos consumption/sign audit",
        "audit_only": True,
        "freeze": {"path": _relative(FREEZE), "sha256": FREEZE_SHA256},
        "evaluator_manifest": {"path": _relative(MANIFEST), "sha256": VERIFIED_MANIFEST_SHA256},
        "source_evidence": source,
        "source_checks": source_checks,
        "routes": {route: report for route, report in zip(ROUTES, public_routes)},
        "aggregate": aggregate,
        "leave_one_route_out": loo,
        "gates": {"all_passed": all_passed, "observed": gates, "route_observed": {route: report["gates"] for route, report in zip(ROUTES, route_reports)}, "presentation": presentation},
        "read_accounting": reads,
        "decision": decision,
        "artifacts": {
            "routes": "phase60_pixel5_intersignal_bias.routes.json",
            "events": "phase60_pixel5_intersignal_bias.events.json",
            "manifest": "phase60_pixel5_intersignal_bias.manifest.json",
        },
    }
    output_root.mkdir(parents=True, exist_ok=False)
    result_path = output_root / "phase60_pixel5_intersignal_bias.json"
    routes_path = output_root / "phase60_pixel5_intersignal_bias.routes.json"
    events_path = output_root / "phase60_pixel5_intersignal_bias.events.json"
    manifest_path = output_root / "phase60_pixel5_intersignal_bias.manifest.json"
    result_payload = _atomic_json(result_path, result)
    routes_payload = _atomic_json(routes_path, {"schema_version": SCHEMA + ".routes", "status": status, "routes": {route: report for route, report in zip(ROUTES, public_routes)}})
    events_payload = _atomic_json(events_path, {"schema_version": SCHEMA + ".events", "status": status, "events": all_events})
    output_manifest = {
        "schema_version": SCHEMA + ".output-manifest",
        "status": "atomic-publish-complete",
        "phase": 60,
        "freeze_sha256": FREEZE_SHA256,
        "evaluator_manifest_sha256": VERIFIED_MANIFEST_SHA256,
        "read_accounting": reads,
        "artifacts": {
            "result": {"path": _relative(result_path), "bytes": len(result_payload), "sha256": _sha256_bytes(result_payload)},
            "routes": {"path": _relative(routes_path), "bytes": len(routes_payload), "sha256": _sha256_bytes(routes_payload)},
            "events": {"path": _relative(events_path), "bytes": len(events_payload), "sha256": _sha256_bytes(events_payload)},
        },
    }
    manifest_payload = _atomic_json(manifest_path, output_manifest)
    result["output_artifact_hashes"] = {
        "result_sha256": _sha256_bytes(result_payload),
        "routes_sha256": _sha256_bytes(routes_payload),
        "events_sha256": _sha256_bytes(events_payload),
        "manifest_sha256": _sha256_bytes(manifest_payload),
    }
    return result


def main(argv: Iterable[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--verify-freeze", action="store_true", help="verify freeze/manifest without raw reads")
    parser.add_argument("--audit", action="store_true", help="run one-shot four-route raw-only audit")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args(list(argv) if argv is not None else None)
    if args.verify_freeze == args.audit:
        parser.error("choose exactly one of --verify-freeze or --audit")
    try:
        freeze = _verify_freeze()
        manifest = _verify_manifest(freeze)
        if args.verify_freeze:
            print("phase60 freeze/evaluator manifest: verified without raw/truth reads")
            return 0
        result = _audit(freeze, manifest, args.output)
        print(json.dumps({
            "status": result["status"],
            "all_gates_passed": result["gates"]["all_passed"],
            "raw_reads": result["read_accounting"]["raw_device_gnss_reads_total"],
        }, sort_keys=True))
        return 0
    except Phase60Error as exc:
        print(f"phase60 fail-closed: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
