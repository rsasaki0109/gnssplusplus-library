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
  ``StreamingRinexWriter`` (every other row is preserved but excluded),
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

from gnss_smartphone_gnss_adapter import (
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
    if args.enable_galileo_e1 and args.broadcast_nav is None:
        fail("--broadcast-nav is required with --enable-galileo-e1")
    if args.broadcast_nav is not None and not args.enable_galileo_e1:
        fail("--broadcast-nav is only valid with --enable-galileo-e1")
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

    dispositions = {name: 0 for name in DISPOSITIONS}
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
            writer = StreamingRinexWriter(
                tmp / "rover.obs", approx, enable_galileo_e1=args.enable_galileo_e1
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

                def classify(line: int, row: dict[str, str]):
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
                    for ln, row, disp, signal, pr, arrival in rows_in_epoch:
                        final_disp = disp
                        if capped and disp == "used":
                            final_disp = "unsupported_signal"  # excluded by --max-epochs
                        dispositions[final_disp] += 1
                        if final_disp == "used":
                            signal_rows[signal] = signal_rows.get(signal, 0) + 1
                            used_rows.append((ln, row, signal, pr, arrival))
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
                    gsdc_rows = [
                        to_gsdc_row(r, ln, signal, pr, arrival)
                        for ln, r, signal, pr, arrival in used_rows
                    ]
                    timestamp = _int(used_rows[0][1]["utcTimeMillis"], "utcTimeMillis", used_rows[0][0])
                    for ln, r, signal, pr, arrival in used_rows:
                        utc_minus_arrival.append(
                            arrival / 1e6 - _int(r["utcTimeMillis"], "utcTimeMillis", ln)
                        )
                    writer.write_epoch(timestamp, gsdc_rows, count)
                    epochs_selected += 1
                    arr = [a for *_x, a in used_rows]
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
                    disp, signal, pr, arrival = classify(line, row)
                    epoch_rows.append((line, row, disp, signal, pr, arrival))
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
            if args.enable_galileo_e1:
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
