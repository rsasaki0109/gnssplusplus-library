#!/usr/bin/env python3
"""Fail-closed adapter for Mimir (TAU) Android Raw.csv / PSR.csv logs.

The Nantes multi-sensor dataset (Zenodo 12566912) logs Android
``GnssMeasurement`` rows with the Mimir app.  The per-sensor ``Raw.csv`` has
*no header row* (the header lives only in the combined ``log_mimir_*.txt``)
and lacks the GSDC ``device_gnss.csv`` convenience columns (``SignalType``,
``RawPseudorangeMeters``, ``ArrivalTimeNanosSinceGpsEpoch``, WLS position).
This adapter

* pins the exact 35-column Mimir header and rejects any other shape,
* derives the Android raw pseudorange / GPST arrival time with the standard
  GnssLogger algorithm,
* maps GPS L1 C/A and (optionally) Galileo E1 rows onto the existing R5
  ``StreamingRinexWriter`` (every other row is preserved but excluded); this
  is the default ``--signal-set legacy-l1-e1`` and its outputs are
  byte-identical to the pre-multi-signal adapter,
* with ``--signal-set multi`` infers the signal of every row from
  ``ConstellationType`` + ``CarrierFrequencyHz`` (GPS L1/L5, Galileo E1/E5a,
  GLONASS G1, BeiDou B1I/B2a/B1C, QZSS L1/L5; see
  ``gnss_smartphone_mimir_signals``) and writes a multi-signal RINEX 3.04,
* accounts for every source row with exactly one terminal disposition,
* converts ``PSR.csv`` barometer samples to GPST,
* publishes all artifacts atomically and hashes inputs and outputs.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from statistics import median

import gnss_smartphone_mimir_signals as sigs
from gnss_smartphone_gnss_adapter import (
    HATCH_WINDOW_SECONDS,
    GALILEO_E1_HZ,
    GALILEO_E1_SIGNAL,
    GPS_L1_HZ,
    SPEED_OF_LIGHT_MPS,
    StreamingRinexWriter,
    ecef_to_geodetic as _ecef_to_geodetic,  # noqa: F401  (re-export for tests)
    fail,
    validate_galileo_navigation,
)

SCHEMA_VERSION = "smartphone-mimir-adapter.v1"
SCHEMA_VERSION_MULTI = "smartphone-mimir-adapter.v2-multisignal"
SIGNAL_SETS = ("legacy-l1-e1", "multi")

MIMIR_RAW_FIELDS = (
    "Raw",
    "utcTimeMillis",
    "TimeNanos",
    "LeapSecond",
    "TimeUncertaintyNanos",
    "FullBiasNanos",
    "BiasNanos",
    "BiasUncertaintyNanos",
    "DriftNanosPerSecond",
    "DriftUncertaintyNanosPerSecond",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "TimeOffsetNanos",
    "State",
    "ReceivedSvTimeNanos",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "PseudorangeRateUncertaintyMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "AccumulatedDeltaRangeUncertaintyMeters",
    "CarrierFrequencyHz",
    "CarrierCycles",
    "CarrierPhase",
    "CarrierPhaseUncertainty",
    "MultipathIndicator",
    "SnrInDb",
    "ConstellationType",
    "AgcDb",
    "BasebandCn0DbHz",
    "FullInterSignalBiasNanos",
    "FullInterSignalBiasUncertaintyNanos",
    "SatelliteInterSignalBiasNanos",
    "SatelliteInterSignalBiasUncertaintyNanos",
)
PSR_FIELD_COUNT = 5  # PSR,utcTimeMillis,elapsedRealtimeNanos,pressure_hPa,accuracy

# Android GnssMeasurement.STATE_* bit flags.
STATE_CODE_LOCK = 1
STATE_TOW_DECODED = 8
STATE_MSEC_AMBIGUOUS = 16
STATE_GAL_E1BC_CODE_LOCK = 1 << 10
STATE_TOW_KNOWN = 1 << 14

WEEK_NS = 604_800 * 10**9
NOMINAL_L1_TOLERANCE_HZ = 1000.0
# Android reports the tracked carrier with a Doppler-scale offset (observed
# +30 Hz); a wider mismatch is a different signal, not a labelling nuance.
MAX_RECEIVED_SV_TIME_UNCERTAINTY_NS = 500.0
MIN_TRAVEL_TIME_S = 0.04
MAX_TRAVEL_TIME_S = 0.14
MIN_PRESSURE_HPA = 300.0
MAX_PRESSURE_HPA = 1100.0
MAX_UTC_OFFSET_SPREAD_S = 2.0
GPS_UTC_LEAP_SECONDS = 18  # nominal; the estimated offset is what is applied

DISPOSITIONS = (
    "used",
    "unsupported_signal",
    "state_not_usable",
    "uncertain_received_sv_time",
    "implausible_travel_time",
    "no_range_fields",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _int(token: str, field: str, line: int) -> int:
    try:
        return int(token)
    except (TypeError, ValueError):
        # Android sometimes serialises integers as ``1.0``; accept only exact.
        try:
            value = float(token)
        except (TypeError, ValueError):
            fail(f"Raw row {line}: invalid integer {field}: {token!r}")
        if not math.isfinite(value) or not value.is_integer():
            fail(f"Raw row {line}: {field} must be an exact integer: {token!r}")
        return int(value)


def _float(token: str, field: str, line: int) -> float:
    try:
        value = float(token)
    except (TypeError, ValueError):
        fail(f"Raw row {line}: invalid number {field}: {token!r}")
    if not math.isfinite(value):
        fail(f"Raw row {line}: {field} must be finite")
    return value


def classify_signal(constellation: str, carrier_hz: float | None) -> str | None:
    """Return the R5 SignalType for a supported row, else ``None``."""

    if carrier_hz is None:
        return None
    if constellation == "1" and abs(carrier_hz - GPS_L1_HZ) <= NOMINAL_L1_TOLERANCE_HZ:
        return "GPS_L1_CA"
    if constellation == "6" and abs(carrier_hz - GALILEO_E1_HZ) <= NOMINAL_L1_TOLERANCE_HZ:
        return GALILEO_E1_SIGNAL
    return None


def state_usable(constellation: str, state: int) -> bool:
    """Whether ReceivedSvTimeNanos is an unambiguous week-time of transmission."""

    if state & STATE_MSEC_AMBIGUOUS:
        return False
    tow_known = bool(state & (STATE_TOW_DECODED | STATE_TOW_KNOWN))
    if constellation == "1":
        return bool(state & STATE_CODE_LOCK) and tow_known
    if constellation == "6":
        return bool(state & (STATE_CODE_LOCK | STATE_GAL_E1BC_CODE_LOCK)) and tow_known
    return False


def arrival_time_ns(row: dict[str, str], line: int) -> tuple[int, float]:
    """GnssLogger: tRxGnss = TimeNanos + TimeOffsetNanos - (FullBias + Bias)."""

    time_nanos = _int(row["TimeNanos"], "TimeNanos", line)
    full_bias = _int(row["FullBiasNanos"], "FullBiasNanos", line)
    bias = _float(row["BiasNanos"] or "0", "BiasNanos", line)
    offset = _float(row["TimeOffsetNanos"] or "0", "TimeOffsetNanos", line)
    whole = time_nanos - full_bias
    frac = offset - bias
    return whole, frac


def pseudorange_from_row(
    row: dict[str, str], line: int
) -> tuple[float, float] | tuple[None, None]:
    """Return (pseudorange_m, arrival_ns_since_gps_epoch) or (None, None)."""

    whole, frac = arrival_time_ns(row, line)
    arrival_ns = whole + frac
    week = math.floor(whole / WEEK_NS)
    tow_rx_ns = (whole - week * WEEK_NS) + frac
    tx_ns = _float(row["ReceivedSvTimeNanos"], "ReceivedSvTimeNanos", line)
    travel_ns = tow_rx_ns - tx_ns
    if travel_ns > WEEK_NS / 2:
        travel_ns -= WEEK_NS
    elif travel_ns < -WEEK_NS / 2:
        travel_ns += WEEK_NS
    travel_s = travel_ns * 1e-9
    if not MIN_TRAVEL_TIME_S <= travel_s <= MAX_TRAVEL_TIME_S:
        return None, None
    return travel_s * SPEED_OF_LIGHT_MPS, arrival_ns


def parse_raw_line(fields: list[str], line: int) -> dict[str, str]:
    if len(fields) != len(MIMIR_RAW_FIELDS):
        fail(
            f"Raw row {line}: expected {len(MIMIR_RAW_FIELDS)} Mimir fields, "
            f"got {len(fields)}"
        )
    row = dict(zip(MIMIR_RAW_FIELDS, fields))
    if row["Raw"] != "Raw":
        fail(f"Raw row {line}: first field must be 'Raw', got {row['Raw']!r}")
    return row


def to_gsdc_row(
    row: dict[str, str], line: int, signal: str, pseudorange: float, arrival_ns: float
) -> dict[str, str]:
    """Map a Mimir row onto the GSDC columns consumed by StreamingRinexWriter."""

    nominal = GPS_L1_HZ if signal == "GPS_L1_CA" else GALILEO_E1_HZ
    return {
        "SignalType": signal,
        "ConstellationType": row["ConstellationType"].strip(),
        "CarrierFrequencyHz": repr(nominal),
        "CodeType": "",
        "Svid": row["Svid"].strip(),
        "RawPseudorangeMeters": repr(pseudorange),
        "ArrivalTimeNanosSinceGpsEpoch": repr(arrival_ns),
        "PseudorangeRateMetersPerSecond": row["PseudorangeRateMetersPerSecond"],
        "Cn0DbHz": row["Cn0DbHz"],
        "AccumulatedDeltaRangeState": row["AccumulatedDeltaRangeState"] or "0",
        "AccumulatedDeltaRangeMeters": row["AccumulatedDeltaRangeMeters"],
        "HardwareClockDiscontinuityCount": row["HardwareClockDiscontinuityCount"],
    }


def read_psr(path: Path) -> tuple[list[tuple[int, int, float, int]], dict[str, int]]:
    rows: list[tuple[int, int, float, int]] = []
    counts = {"rows": 0, "kept": 0, "dropped_out_of_range": 0}
    last_utc = None
    with path.open(encoding="utf-8", newline="") as handle:
        for line, fields in enumerate(csv.reader(handle), start=1):
            if not fields:
                continue
            if len(fields) != PSR_FIELD_COUNT or fields[0] != "PSR":
                fail(f"PSR row {line}: expected PSR + {PSR_FIELD_COUNT - 1} fields")
            counts["rows"] += 1
            utc_ms = _int(fields[1], "utcTimeMillis", line)
            elapsed_ns = _int(fields[2], "elapsedRealtimeNanos", line)
            pressure = _float(fields[3], "pressure_hPa", line)
            accuracy = _int(fields[4], "accuracy", line)
            if last_utc is not None and utc_ms < last_utc:
                fail(f"PSR row {line}: utcTimeMillis moved backwards")
            last_utc = utc_ms
            if not MIN_PRESSURE_HPA <= pressure <= MAX_PRESSURE_HPA:
                counts["dropped_out_of_range"] += 1
                continue
            rows.append((utc_ms, elapsed_ns, pressure, accuracy))
            counts["kept"] += 1
    if not rows:
        fail(f"no usable pressure samples in {path}")
    return rows, counts


def scan_glonass_channels(raw: Path) -> dict[int, set[int]]:
    """Pre-scan: GLONASS slot -> set of carrier-derived frequency channels.

    The RINEX header (GLONASS SLOT / FRQ #) must be written before the first
    epoch, so the channel map is established in a separate read-only pass.
    """

    channels: dict[int, set[int]] = {}
    with raw.open(encoding="utf-8", newline="") as handle:
        for line, fields in enumerate(csv.reader(handle), start=1):
            if not fields:
                continue
            row = parse_raw_line(fields, line)
            if row["ConstellationType"].strip() != "3" or not row["CarrierFrequencyHz"].strip():
                continue
            fcn = sigs.glonass_fcn_from_carrier(
                _float(row["CarrierFrequencyHz"], "CarrierFrequencyHz", line)
            )
            if fcn is not None:
                channels.setdefault(_int(row["Svid"], "Svid", line), set()).add(fcn)
    return channels


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--raw", type=Path, required=True, help="Mimir Raw.csv (headerless)")
    parser.add_argument("--psr", type=Path, required=True, help="Mimir PSR.csv barometer log")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--device-model", default="Pixel 7")
    parser.add_argument("--source-url", required=True)
    parser.add_argument("--source-terms", required=True)
    parser.add_argument(
        "--approx-llh",
        required=True,
        help="lat_deg,lon_deg,height_m RINEX seed (coarse scenario coordinate)",
    )
    parser.add_argument("--enable-galileo-e1", action="store_true")
    parser.add_argument("--broadcast-nav", type=Path)
    parser.add_argument(
        "--signal-set",
        choices=SIGNAL_SETS,
        default="legacy-l1-e1",
        help="legacy-l1-e1 (default, byte-identical to the original adapter) or "
        "multi (carrier-frequency inferred multi-constellation/multi-band)",
    )
    parser.add_argument(
        "--enable-signals",
        help="multi only: comma-separated subset of signal names "
        f"({','.join(sigs.SIGNAL_BY_NAME)}); default all",
    )
    parser.add_argument(
        "--hatch-window-s",
        type=int,
        choices=HATCH_WINDOW_SECONDS,
        help="R5 promoted Hatch C1C smoothing of Galileo E1 (window seconds); "
        "unchanged R5 HatchSmoother, applies to Galileo E1 only",
    )
    parser.add_argument("--max-epochs", type=int, default=-1)
    return parser.parse_args()


def llh_to_ecef(lat_deg: float, lon_deg: float, h: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    return (
        (n + h) * math.cos(lat) * math.cos(lon),
        (n + h) * math.cos(lat) * math.sin(lon),
        (n * (1.0 - e2) + h) * math.sin(lat),
    )


def main() -> int:
    args = parse_args()
    multi = args.signal_set == "multi"
    if multi:
        if args.enable_galileo_e1:
            fail("--enable-galileo-e1 is implied by --signal-set multi; do not pass both")
        if args.broadcast_nav is None:
            fail("--broadcast-nav (mixed BRDC) is required with --signal-set multi")
    else:
        if args.enable_galileo_e1 and args.broadcast_nav is None:
            fail("--broadcast-nav is required with --enable-galileo-e1")
        if args.broadcast_nav is not None and not args.enable_galileo_e1:
            fail("--broadcast-nav is only valid with --enable-galileo-e1")
        if args.enable_signals is not None:
            fail("--enable-signals is only valid with --signal-set multi")
        if args.hatch_window_s is not None and not args.enable_galileo_e1:
            fail("--hatch-window-s requires --enable-galileo-e1 (or --signal-set multi)")
    enabled_names = set(sigs.SIGNAL_BY_NAME)
    if args.enable_signals is not None:
        enabled_names = {n.strip() for n in args.enable_signals.split(",") if n.strip()}
        unknown = sorted(enabled_names - set(sigs.SIGNAL_BY_NAME))
        if unknown or not enabled_names:
            fail(f"--enable-signals has unknown/empty signal names: {unknown}")
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")
    try:
        lat_s, lon_s, h_s = args.approx_llh.split(",")
        approx = llh_to_ecef(float(lat_s), float(lon_s), float(h_s))
    except ValueError:
        fail("--approx-llh must be lat_deg,lon_deg,height_m")
    if not all(math.isfinite(v) for v in approx):
        fail("--approx-llh must be finite")
    for path in (args.raw, args.psr):
        if not path.is_file():
            fail(f"missing input: {path}")

    out = args.output_dir
    out.mkdir(parents=True, exist_ok=True)
    final_paths = {
        "observations": out / "observations.csv",
        "rinex": out / "rover.obs",
        "baro": out / "baro.csv",
        "summary": out / "summary.json",
    }

    dispositions = {name: 0 for name in (sigs.MULTI_DISPOSITIONS if multi else DISPOSITIONS)}
    rejected_detail: dict[str, dict[str, int]] = {}
    nav_index = None
    glonass_channels: dict[int, int] = {}
    glonass_scan: dict[int, set[int]] = {}
    nav_uncovered: dict[str, int] = {}
    nav_max_age: dict[str, float] = {}
    enabled_specs = tuple(sp for sp in sigs.SIGNALS if sp.name in enabled_names)
    if multi:
        nav_index = sigs.NavigationIndex.load(args.broadcast_nav)
        glonass_scan = scan_glonass_channels(args.raw)
        for slot, ks in glonass_scan.items():
            nav_ks = nav_index.glonass_channels.get(slot)
            if len(ks) == 1 and nav_ks and next(iter(ks)) in nav_ks:
                glonass_channels[slot] = next(iter(ks))
    signal_rows: dict[str, int] = {}
    constellation_rows: dict[str, int] = {}
    state_rows_unusable: dict[str, int] = {}
    clock_counts: list[int] = []
    epochs_total = 0
    epochs_selected = 0
    rows_total = 0
    utc_minus_arrival: list[float] = []
    first_arrival_ns: float | None = None
    last_arrival_ns: float | None = None
    last_time_nanos: int | None = None

    with tempfile.TemporaryDirectory(prefix=".mimir-adapter-", dir=str(out)) as tmp_name:
        tmp = Path(tmp_name)
        writer: StreamingRinexWriter | None = None
        try:
            if multi:
                writer = sigs.MultiSignalRinexWriter(
                    tmp / "rover.obs", approx, enabled_specs,
                    glonass_channels if "GLO_G1_CA" in enabled_names else {},
                    hatch_window_s=args.hatch_window_s,
                )
            else:
                writer = StreamingRinexWriter(
                    tmp / "rover.obs", approx, enable_galileo_e1=args.enable_galileo_e1,
                    hatch_window_s=args.hatch_window_s,
                )
            with args.raw.open(encoding="utf-8", newline="") as raw_handle, (
                tmp / "observations.csv"
            ).open("w", encoding="utf-8", newline="") as norm_handle:
                norm = csv.writer(norm_handle, lineterminator="\n")
                norm.writerow(
                    (*MIMIR_RAW_FIELDS, "disposition", "SignalType", "RawPseudorangeMeters",
                     "ArrivalTimeNanosSinceGpsEpoch")
                )
                current_key: str | None = None

                def reject(disp: str, detail: str):
                    rejected_detail.setdefault(disp, {})
                    rejected_detail[disp][detail] = rejected_detail[disp].get(detail, 0) + 1

                def classify_multi_row(line: int, row: dict[str, str]):
                    """Returns (disposition, signal, pr, arrival, extra)."""
                    constellation = row["ConstellationType"].strip()
                    constellation_rows[constellation] = constellation_rows.get(constellation, 0) + 1
                    carrier = row["CarrierFrequencyHz"].strip()
                    carrier_hz = _float(carrier, "CarrierFrequencyHz", line) if carrier else None
                    spec, fcn, reason = sigs.classify_multi(constellation, carrier_hz)
                    cname = sigs.CONSTELLATION_NAMES.get(constellation, f"type{constellation}")
                    if spec is None:
                        mhz = "none" if carrier_hz is None else f"{carrier_hz / 1e6:.2f}MHz"
                        reject(reason, f"{cname}@{mhz}")
                        return reason, "", None, None, None
                    if spec.name not in enabled_names:
                        reject("signal_not_enabled", spec.name)
                        return "signal_not_enabled", spec.name, None, None, None
                    svid = _int(row["Svid"], "Svid", line)
                    prn = sigs.prn_from_svid(spec, svid)
                    if prn is None:
                        reject("invalid_svid", f"{spec.name}:svid{svid}")
                        return "invalid_svid", spec.name, None, None, None
                    if spec.name == "GLO_G1_CA":
                        if prn not in nav_index.glonass_channels:
                            reject("no_navigation", f"{spec.name}:R{prn:02d}")
                            nav_uncovered[spec.name] = nav_uncovered.get(spec.name, 0) + 1
                            return "no_navigation", spec.name, None, None, None
                        if glonass_channels.get(prn) != fcn:
                            reject("glonass_fcn_conflict", f"R{prn:02d}")
                            return "glonass_fcn_conflict", spec.name, None, None, None
                    state = _int(row["State"], "State", line)
                    if not sigs.state_usable_multi(constellation, state):
                        key = f"{constellation}:{state}"
                        state_rows_unusable[key] = state_rows_unusable.get(key, 0) + 1
                        return "state_not_usable", spec.name, None, None, None
                    unc = _float(row["ReceivedSvTimeUncertaintyNanos"], "ReceivedSvTimeUncertaintyNanos", line)
                    if unc > MAX_RECEIVED_SV_TIME_UNCERTAINTY_NS:
                        return "uncertain_received_sv_time", spec.name, None, None, None
                    if not row["ReceivedSvTimeNanos"].strip() or not row["Cn0DbHz"].strip() or not row[
                        "PseudorangeRateMetersPerSecond"
                    ].strip():
                        return "no_range_fields", spec.name, None, None, None
                    whole, frac = arrival_time_ns(row, line)
                    tx_ns = _float(row["ReceivedSvTimeNanos"], "ReceivedSvTimeNanos", line)
                    travel_s = sigs.travel_time_s(constellation, whole, frac, tx_ns)
                    if not MIN_TRAVEL_TIME_S <= travel_s <= MAX_TRAVEL_TIME_S:
                        return "implausible_travel_time", spec.name, None, None, None
                    arrival = whole + frac
                    covered, age = nav_index.covers(
                        spec.rinex_system, prn, sigs.gpst_to_datetime(arrival / 1e9)
                    )
                    if not covered:
                        reject("no_navigation", f"{spec.name}:{spec.rinex_system}{prn:02d}")
                        nav_uncovered[spec.name] = nav_uncovered.get(spec.name, 0) + 1
                        return "no_navigation", spec.name, None, None, None
                    nav_max_age[spec.name] = max(nav_max_age.get(spec.name, 0.0), age)
                    return "used", spec.name, travel_s * SPEED_OF_LIGHT_MPS, arrival, (spec, prn, carrier_hz)

                def classify(line: int, row: dict[str, str]):
                    if multi:
                        return classify_multi_row(line, row)
                    return (*classify_legacy(line, row), None)

                def classify_legacy(line: int, row: dict[str, str]):
                    constellation = row["ConstellationType"].strip()
                    constellation_rows[constellation] = constellation_rows.get(constellation, 0) + 1
                    carrier = row["CarrierFrequencyHz"].strip()
                    carrier_hz = _float(carrier, "CarrierFrequencyHz", line) if carrier else None
                    signal = classify_signal(constellation, carrier_hz)
                    if signal == GALILEO_E1_SIGNAL and not args.enable_galileo_e1:
                        signal = None
                    if signal is None:
                        return "unsupported_signal", "", None, None
                    state = _int(row["State"], "State", line)
                    if not state_usable(constellation, state):
                        key = f"{constellation}:{state}"
                        state_rows_unusable[key] = state_rows_unusable.get(key, 0) + 1
                        return "state_not_usable", signal, None, None
                    unc = _float(row["ReceivedSvTimeUncertaintyNanos"], "ReceivedSvTimeUncertaintyNanos", line)
                    if unc > MAX_RECEIVED_SV_TIME_UNCERTAINTY_NS:
                        return "uncertain_received_sv_time", signal, None, None
                    if not row["ReceivedSvTimeNanos"].strip() or not row["Cn0DbHz"].strip() or not row[
                        "PseudorangeRateMetersPerSecond"
                    ].strip():
                        return "no_range_fields", signal, None, None
                    pr, arrival = pseudorange_from_row(row, line)
                    if pr is None:
                        return "implausible_travel_time", signal, None, None
                    return "used", signal, pr, arrival

                def emit_epoch(rows_in_epoch) -> None:
                    nonlocal epochs_total, epochs_selected, first_arrival_ns, last_arrival_ns
                    epochs_total += 1
                    capped = args.max_epochs > 0 and epochs_selected >= args.max_epochs
                    used_rows = []
                    for ln, row, disp, signal, pr, arrival, extra in rows_in_epoch:
                        final_disp = disp
                        if capped and disp == "used":
                            # excluded by --max-epochs
                            final_disp = "excluded_by_max_epochs" if multi else "unsupported_signal"
                        dispositions[final_disp] += 1
                        if final_disp == "used":
                            signal_rows[signal] = signal_rows.get(signal, 0) + 1
                            used_rows.append((ln, row, signal, pr, arrival, extra))
                        norm.writerow(
                            (
                                *[row[f] for f in MIMIR_RAW_FIELDS],
                                final_disp,
                                signal,
                                "" if pr is None else repr(pr),
                                "" if arrival is None else repr(arrival),
                            )
                        )
                    if not used_rows:
                        return
                    counts = {
                        _int(r["HardwareClockDiscontinuityCount"], "HardwareClockDiscontinuityCount", ln)
                        for ln, r, *_ in used_rows
                    }
                    if len(counts) != 1:
                        fail("inconsistent hardware clock discontinuity count within an epoch")
                    count = counts.pop()
                    if clock_counts and count < clock_counts[-1]:
                        fail("hardware clock discontinuity count moved backwards")
                    clock_counts.append(count)
                    timestamp = _int(used_rows[0][1]["utcTimeMillis"], "utcTimeMillis", used_rows[0][0])
                    for ln, r, signal, pr, arrival, _extra in used_rows:
                        utc_minus_arrival.append(
                            arrival / 1e6 - _int(r["utcTimeMillis"], "utcTimeMillis", ln)
                        )
                    if multi:
                        records = [
                            sigs.ObservationRecord(
                                spec=extra[0], prn=extra[1], pseudorange_m=pr, arrival_ns=arrival,
                                carrier_hz=extra[2],
                                doppler_rate_mps=_float(r["PseudorangeRateMetersPerSecond"], "PseudorangeRateMetersPerSecond", ln),
                                cn0_dbhz=_float(r["Cn0DbHz"], "Cn0DbHz", ln),
                                adr_state=_int(r["AccumulatedDeltaRangeState"] or "0", "AccumulatedDeltaRangeState", ln),
                                adr_token=r["AccumulatedDeltaRangeMeters"].strip(),
                            )
                            for ln, r, signal, pr, arrival, extra in used_rows
                        ]
                        writer.write_epoch(timestamp, records, count)
                    else:
                        gsdc_rows = [
                            to_gsdc_row(r, ln, signal, pr, arrival)
                            for ln, r, signal, pr, arrival, _extra in used_rows
                        ]
                        writer.write_epoch(timestamp, gsdc_rows, count)
                    epochs_selected += 1
                    arr = [u[4] for u in used_rows]
                    if first_arrival_ns is None:
                        first_arrival_ns = min(arr)
                    last_arrival_ns = max(arr)

                epoch_rows: list = []
                for line, fields in enumerate(csv.reader(raw_handle), start=1):
                    if not fields:
                        continue
                    row = parse_raw_line(fields, line)
                    rows_total += 1
                    time_nanos = _int(row["TimeNanos"], "TimeNanos", line)
                    if last_time_nanos is not None and time_nanos < last_time_nanos:
                        fail(f"Raw row {line}: TimeNanos moved backwards")
                    key = row["TimeNanos"]
                    if current_key is not None and key != current_key:
                        emit_epoch(epoch_rows)
                        epoch_rows = []
                    current_key = key
                    last_time_nanos = time_nanos
                    disp, signal, pr, arrival, extra = classify(line, row)
                    epoch_rows.append((line, row, disp, signal, pr, arrival, extra))
                if epoch_rows:
                    emit_epoch(epoch_rows)

            if rows_total == 0:
                fail(f"no data rows in {args.raw}")
            if sum(dispositions.values()) != rows_total:
                fail("row accounting error: dispositions do not cover every source row")
            if epochs_selected == 0:
                fail("no supported observations with raw pseudoranges")
            writer.close()
            rinex_summary = writer.summary(final_paths["rinex"])
            navigation_summary = None
            if multi:
                navigation_summary = {
                    "path": str(args.broadcast_nav),
                    "sha256": sha256_file(args.broadcast_nav),
                    "record_prns_by_system": {
                        system: sorted(p for (s_, p) in nav_index.records if s_ == system)
                        for system in sorted({s_ for (s_, _p) in nav_index.records})
                    },
                    "glonass_nav_channels": {
                        str(k): sorted(v) for k, v in sorted(nav_index.glonass_channels.items())
                    },
                    "glonass_carrier_channels_observed": {
                        str(k): sorted(v) for k, v in sorted(glonass_scan.items())
                    },
                    "glonass_channels_in_rinex_header": {
                        str(k): v for k, v in sorted(glonass_channels.items())
                    },
                    "rows_without_navigation_by_signal": dict(sorted(nav_uncovered.items())),
                    "max_nearest_record_age_s_by_signal": dict(sorted(nav_max_age.items())),
                    "max_allowed_record_age_s": sigs.NAV_MAX_AGE_S,
                    "source_policy": "row-level: a measurement without a broadcast record "
                    "inside the age limit is rejected as no_navigation (accounted, not dropped)",
                }
            elif args.enable_galileo_e1:
                if not writer.galileo_epoch_prns:
                    fail("--enable-galileo-e1 found no Galileo E1 observations")
                navigation_summary = validate_galileo_navigation(
                    args.broadcast_nav, writer.galileo_epoch_prns
                )

            # --- barometer: PSR utc (phone system clock) -> GPST -------------
            offsets = sorted(utc_minus_arrival)
            n = len(offsets)
            spread = (offsets[int(0.95 * (n - 1))] - offsets[int(0.05 * (n - 1))]) / 1000.0
            if spread > MAX_UTC_OFFSET_SPREAD_S:
                fail(
                    "phone utcTimeMillis is inconsistent with GNSS arrival time "
                    f"(P5-P95 spread {spread:.3f} s)"
                )
            utc_to_gpst_offset_ms = median(offsets)  # gpst_ms = utc_ms + offset
            psr_rows, psr_counts = read_psr(args.psr)
            with (tmp / "baro.csv").open("w", encoding="utf-8", newline="") as baro_handle:
                baro = csv.writer(baro_handle, lineterminator="\n")
                baro.writerow(
                    ("gps_week", "gps_tow_s", "pressure_hpa", "sensor_accuracy",
                     "utc_time_millis", "elapsed_realtime_ns")
                )
                for utc_ms, elapsed_ns, pressure, accuracy in psr_rows:
                    gpst_s = (utc_ms + utc_to_gpst_offset_ms) / 1000.0
                    week = int(gpst_s // 604800.0)
                    tow = gpst_s - week * 604800.0
                    baro.writerow(
                        (week, f"{tow:.3f}", f"{pressure:.5f}", accuracy, utc_ms, elapsed_ns)
                    )

            summary = {
                "schema_version": SCHEMA_VERSION,
                "dataset": {
                    "id": args.dataset_id,
                    "device_model": args.device_model,
                    "source_url": args.source_url,
                    "source_terms": args.source_terms,
                },
                "inputs": {
                    "raw": {"path": str(args.raw), "sha256": sha256_file(args.raw)},
                    "psr": {"path": str(args.psr), "sha256": sha256_file(args.psr)},
                },
                "mimir_header_contract": list(MIMIR_RAW_FIELDS),
                "observations": {
                    "rows": rows_total,
                    "epochs": epochs_total,
                    "epochs_with_usable_rows": epochs_selected,
                    "row_dispositions": dispositions,
                    "used_signal_rows": dict(sorted(signal_rows.items())),
                    "constellation_rows": dict(sorted(constellation_rows.items())),
                    "state_not_usable_constellation_state": dict(
                        sorted(state_rows_unusable.items())
                    ),
                    "first_gpst_arrival_ns": first_arrival_ns,
                    "last_gpst_arrival_ns": last_arrival_ns,
                    "hardware_clock_discontinuity_counts": sorted(set(clock_counts)),
                    "signal_policy": ["GPS_L1_CA"]
                    + ([GALILEO_E1_SIGNAL] if args.enable_galileo_e1 else []),
                    "carrier_frequency_normalisation": (
                        f"within {NOMINAL_L1_TOLERANCE_HZ:.0f} Hz of nominal; "
                        "nominal written to RINEX (Android reports +~30 Hz)"
                    ),
                    "pseudorange_policy": {
                        "formula": "(tRxGnss - ReceivedSvTime) * c, week-rollover wrapped",
                        "travel_time_window_s": [MIN_TRAVEL_TIME_S, MAX_TRAVEL_TIME_S],
                        "max_received_sv_time_uncertainty_ns": MAX_RECEIVED_SV_TIME_UNCERTAINTY_NS,
                    },
                },
                "barometer": {
                    "pressure_samples": psr_counts,
                    "utc_to_gpst_offset_ms": utc_to_gpst_offset_ms,
                    "utc_to_gpst_offset_nominal_ms": -315_964_800_000 + GPS_UTC_LEAP_SECONDS * 1000,
                    "phone_clock_minus_true_utc_s": (-315_964_800_000 + GPS_UTC_LEAP_SECONDS * 1000 - utc_to_gpst_offset_ms) / 1000.0,
                    "utc_arrival_offset_p5_p95_spread_s": spread,
                },
                "rinex_seed": {
                    "source": "scenario coarse coordinate (notes.txt); not a truth input",
                    "approx_llh": args.approx_llh,
                },
                "native_observation_adapter": rinex_summary,
                "navigation": navigation_summary,
            }
            if multi:
                summary["schema_version"] = SCHEMA_VERSION_MULTI
                obs = summary["observations"]
                obs["signal_set"] = "multi"
                obs["signal_policy"] = [sp.name for sp in enabled_specs]
                obs["signal_inference"] = (
                    "ConstellationType + CarrierFrequencyHz (Mimir Raw.csv has no "
                    "CodeType/SignalType); table in gnss_smartphone_mimir_signals.SIGNALS"
                )
                obs["carrier_frequency_normalisation"] = (
                    f"within {sigs.FREQ_TOLERANCE_HZ:.0f} Hz of nominal (GLONASS: of "
                    "1602 MHz + k*562.5 kHz); actual carrier used for wavelength"
                )
                obs["pseudorange_policy"]["formula"] = (
                    "(tRxGnss - ReceivedSvTime) * c; GPS/GAL/QZS week-rollover wrapped, "
                    "BeiDou tRx shifted by -14 s (BDT), GLONASS tRx = (tow - 18 s + 3 h) mod 1 day"
                )
                obs["rejected_rows_detail"] = {
                    k: dict(sorted(v.items())) for k, v in sorted(rejected_detail.items())
                }
                obs["signal_table"] = {
                    sp.name: {
                        "constellation_type": sp.constellation,
                        "rinex_system": sp.rinex_system,
                        "nominal_hz": sp.nominal_hz,
                        "rinex_obs_codes": list(sp.obs_codes),
                        "note": sp.note,
                    }
                    for sp in sigs.SIGNALS
                }
                obs["hatch_window_s"] = args.hatch_window_s
            (tmp / "summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            summary["artifact_sha256"] = {
                name: sha256_file(tmp / path.name) for name, path in final_paths.items()
                if name != "summary"
            }
            (tmp / "summary.json").write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
            )
            for name, final in final_paths.items():
                os.replace(tmp / final.name, final)
        finally:
            if writer is not None:
                writer.close()
    print(f"Smartphone Mimir adapter complete: {final_paths['summary']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
