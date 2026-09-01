#!/usr/bin/env python3
"""Validate and normalize Google Smartphone Decimeter Challenge GNSS CSVs."""

from __future__ import annotations

import argparse
import csv
from collections.abc import Iterator
from contextlib import contextmanager
from datetime import datetime, timedelta, timezone
import hashlib
import json
import math
import os
from pathlib import Path
from statistics import median
import tempfile


SCHEMA_VERSION = "smartphone-gnss-adapter.v1"
TEST_AUTHORIZATION_SCHEMA = "smartphone-r5-gsdc2023-native-fgo-test-authorization.v1"
TEST_AUTHORIZATION_MANIFEST_SCHEMA = (
    "smartphone-r5-gsdc2023-native-fgo-test-authorization-manifest.v1"
)
GALILEO_E1_SIGNAL = "GAL_E1_C_P"
GALILEO_E1_HZ = 1_575_420_000.0
GALILEO_PRN_MIN = 1
GALILEO_PRN_MAX = 36
NAVIGATION_MAX_AGE_SECONDS = 4 * 60 * 60

# These are deliberately exact source contracts.  A source row that claims a
# supported SignalType but disagrees with its constellation, frequency, or
# CodeType is rejected instead of being silently re-labelled in RINEX.
SIGNAL_CONTRACTS = {
    "GPS_L1_CA": {
        "constellation_type": "1",
        "frequency_hz": 1_575_420_000.0,
        "rinex_system": "G",
        "rinex_tracking": "1C",
        "rinex_observation_types": ["C1C", "L1C", "D1C", "S1C"],
    },
    GALILEO_E1_SIGNAL: {
        "constellation_type": "6",
        "frequency_hz": GALILEO_E1_HZ,
        "rinex_system": "E",
        "rinex_tracking": "1C",
        "rinex_observation_types": ["C1C", "L1C", "D1C", "S1C"],
    },
}
DEVICE_REQUIRED = (
    "MessageType",
    "utcTimeMillis",
    "TimeNanos",
    "FullBiasNanos",
    "HardwareClockDiscontinuityCount",
    "Svid",
    "State",
    "ReceivedSvTimeNanos",
    "ReceivedSvTimeUncertaintyNanos",
    "Cn0DbHz",
    "PseudorangeRateMetersPerSecond",
    "AccumulatedDeltaRangeState",
    "AccumulatedDeltaRangeMeters",
    "AccumulatedDeltaRangeUncertaintyMeters",
    "CarrierFrequencyHz",
    "ConstellationType",
    "CodeType",
    "SignalType",
    "ArrivalTimeNanosSinceGpsEpoch",
    "RawPseudorangeMeters",
    "RawPseudorangeUncertaintyMeters",
    "WlsPositionXEcefMeters",
    "WlsPositionYEcefMeters",
    "WlsPositionZEcefMeters",
)
TRUTH_REQUIRED = (
    "MessageType",
    "Provider",
    "LatitudeDegrees",
    "LongitudeDegrees",
    "AltitudeMeters",
    "UnixTimeMillis",
)
GPS_EPOCH = datetime(1980, 1, 6)
GPS_L1_HZ = 1_575_420_000.0
SPEED_OF_LIGHT_MPS = 299_792_458.0
HATCH_WINDOW_SECONDS = (10, 30, 60)
ANDROID_ADR_VALID = 1
ANDROID_ADR_RESET = 2
ANDROID_ADR_CYCLE_SLIP = 4
HATCH_MAX_EPOCH_GAP_MS = 1_500


def fail(message: str) -> "NoReturn":
    raise SystemExit(message)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def integer(token: str, field: str, row_number: int) -> int:
    try:
        value = float(token)
    except (TypeError, ValueError):
        fail(f"row {row_number}: invalid {field}: {token!r}")
    if not math.isfinite(value) or not value.is_integer():
        fail(f"row {row_number}: {field} must be a finite integer")
    return int(value)


def finite(token: str, field: str, row_number: int) -> float:
    try:
        value = float(token)
    except (TypeError, ValueError):
        fail(f"row {row_number}: invalid {field}: {token!r}")
    if not math.isfinite(value):
        fail(f"row {row_number}: {field} must be finite")
    return value


@contextmanager
def open_csv_reader(
    path: Path, required: tuple[str, ...]
) -> Iterator[tuple[list[str], csv.DictReader]]:
    if not path.is_file():
        fail(f"missing input: {path}")
    with path.open(encoding="utf-8-sig", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = list(reader.fieldnames or ())
        if len(fields) != len(set(fields)):
            fail(f"duplicate CSV fields in {path}")
        missing = [field for field in required if field not in fields]
        if missing:
            fail(f"missing required fields in {path}: {', '.join(missing)}")
        yield fields, reader


def ecef_to_geodetic(x: float, y: float, z: float) -> tuple[float, float, float]:
    a = 6378137.0
    e2 = 6.6943799901413165e-3
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    height = 0.0
    for _ in range(10):
        sin_lat = math.sin(lat)
        radius = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        height = p / max(math.cos(lat), 1e-15) - radius
        next_lat = math.atan2(z, p * (1.0 - e2 * radius / (radius + height)))
        if abs(next_lat - lat) < 1e-14:
            lat = next_lat
            break
        lat = next_lat
    return math.degrees(lat), math.degrees(lon), height


def horizontal_error_m(lat: float, lon: float, truth_lat: float, truth_lon: float) -> float:
    radius = 6378137.0
    dlat = math.radians(lat - truth_lat)
    dlon = math.radians(lon - truth_lon)
    mean_lat = math.radians((lat + truth_lat) / 2.0)
    return radius * math.hypot(dlat, math.cos(mean_lat) * dlon)


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def rinex_header_line(content: str, label: str) -> str:
    return f"{content[:60]:<60}{label}\n"


def rinex_value(value: float | None) -> str:
    return " " * 16 if value is None else f"{value:14.3f}  "


class HatchSmoother:
    """Truth-free Hatch code smoothing for one explicitly selected signal.

    Android's ``AccumulatedDeltaRangeMeters`` is a range-scale quantity.  The
    update below therefore uses its meter delta directly, while the RINEX
    carrier field continues to receive the same meters-to-cycles conversion as
    the unsmoothed adapter.  State is keyed by RINEX system, PRN, and source
    signal so observations from different signals can never share an arc.

    The development route is one-Hz data.  ``window_s`` consequently denotes
    the maximum number of consecutive one-second samples.  A non-contiguous
    epoch, Android ADR reset/cycle-slip/invalid state, missing ADR, or a
    hardware clock discontinuity starts a new arc and emits the raw code for
    that row.  No truth or reference data is read here.
    """

    def __init__(self, window_s: int) -> None:
        if window_s not in HATCH_WINDOW_SECONDS:
            raise ValueError(f"unsupported Hatch window: {window_s}")
        self.window_s = window_s
        self._states: dict[tuple[str, int, str], dict[str, float | int]] = {}
        self._reset_counts = {
            "invalid_adr_state": 0,
            "adr_reset": 0,
            "cycle_slip": 0,
            "missing_or_out_of_range_adr": 0,
            "epoch_gap": 0,
            "hardware_clock_discontinuity": 0,
        }
        self._arcs_started = 0
        self._updates = 0
        self._max_continuous_samples = 0

    def _clear(self, key: tuple[str, int, str], reason: str) -> None:
        if key in self._states:
            count = int(self._states[key]["count"])
            self._max_continuous_samples = max(self._max_continuous_samples, count)
            del self._states[key]
        self._reset_counts[reason] += 1

    def update(
        self,
        *,
        key: tuple[str, int, str],
        timestamp_ms: int,
        pseudorange_m: float,
        adr_token: str,
        adr_state: int,
        clock_count: int,
    ) -> float:
        adr_m: float | None = None
        if adr_token:
            try:
                candidate = float(adr_token)
            except (TypeError, ValueError):
                candidate = math.nan
            if math.isfinite(candidate) and abs(candidate) < 1e9:
                adr_m = candidate

        # Check reset/slip bits before validity: a row carrying both bits is
        # still a boundary, and must not contribute an ADR delta.
        if adr_state & ANDROID_ADR_CYCLE_SLIP:
            self._clear(key, "cycle_slip")
            return pseudorange_m
        if adr_state & ANDROID_ADR_RESET:
            self._clear(key, "adr_reset")
            return pseudorange_m
        if not adr_state & ANDROID_ADR_VALID:
            self._clear(key, "invalid_adr_state")
            return pseudorange_m
        if adr_m is None:
            self._clear(key, "missing_or_out_of_range_adr")
            return pseudorange_m

        previous = self._states.get(key)
        if previous is not None:
            elapsed_ms = timestamp_ms - int(previous["timestamp_ms"])
            if elapsed_ms <= 0 or elapsed_ms > HATCH_MAX_EPOCH_GAP_MS:
                self._clear(key, "epoch_gap")
                previous = None
            elif clock_count != int(previous["clock_count"]):
                self._clear(key, "hardware_clock_discontinuity")
                previous = None

        if previous is None:
            self._states[key] = {
                "timestamp_ms": timestamp_ms,
                "adr_m": adr_m,
                "smoothed_m": pseudorange_m,
                "clock_count": clock_count,
                "count": 1,
            }
            self._arcs_started += 1
            self._max_continuous_samples = max(self._max_continuous_samples, 1)
            return pseudorange_m

        count = min(self.window_s, int(previous["count"]) + 1)
        predicted_m = float(previous["smoothed_m"]) + adr_m - float(previous["adr_m"])
        smoothed_m = ((count - 1) * predicted_m + pseudorange_m) / count
        self._states[key] = {
            "timestamp_ms": timestamp_ms,
            "adr_m": adr_m,
            "smoothed_m": smoothed_m,
            "clock_count": clock_count,
            "count": count,
        }
        self._updates += 1
        self._max_continuous_samples = max(self._max_continuous_samples, count)
        return smoothed_m

    def summary(self) -> dict[str, object]:
        for state in self._states.values():
            self._max_continuous_samples = max(
                self._max_continuous_samples, int(state["count"])
            )
        return {
            "enabled": True,
            "signal": GALILEO_E1_SIGNAL,
            "window_seconds": self.window_s,
            "source_units": "AccumulatedDeltaRangeMeters is meters",
            "formula": "S_t=((N-1)*(S_(t-1)+ADR_t-ADR_(t-1))+P_t)/N",
            "state_policy": {
                "valid_bit": ANDROID_ADR_VALID,
                "reset_bit": ANDROID_ADR_RESET,
                "cycle_slip_bit": ANDROID_ADR_CYCLE_SLIP,
                "gap_reset_threshold_ms": HATCH_MAX_EPOCH_GAP_MS,
                "clock_discontinuity_resets": True,
                "key": "RINEX system + PRN + SignalType",
                "reset_row_emits": "raw pseudorange",
            },
            "arcs_started": self._arcs_started,
            "updates": self._updates,
            "max_continuous_samples": self._max_continuous_samples,
            "reset_counts": dict(self._reset_counts),
        }


class StreamingRinexWriter:
    """Write one selected device epoch at a time without retaining raw rows."""

    def __init__(
        self,
        path: Path,
        approximate_position: tuple[float, float, float],
        enable_galileo_e1: bool = False,
        hatch_window_s: int | None = None,
    ) -> None:
        self.path = path
        self.enable_galileo_e1 = enable_galileo_e1
        if hatch_window_s is not None and not enable_galileo_e1:
            raise ValueError("Hatch smoothing requires Galileo E1")
        self.hatch_window_s = hatch_window_s
        self._hatch = HatchSmoother(hatch_window_s) if hatch_window_s is not None else None
        self._handle = path.open("w", encoding="ascii", newline="")
        self._source_rows = 0
        self._epochs = 0
        self._signal_rows: dict[str, int] = {}
        self._galileo_epoch_prns: dict[int, set[int]] = {}
        self._write_header(approximate_position)

    def _write_header(self, approximate_position: tuple[float, float, float]) -> None:
        handle = self._handle
        file_system = "M" if self.enable_galileo_e1 else "G"
        handle.write(rinex_header_line(f"     3.04           OBSERVATION DATA    {file_system}", "RINEX VERSION / TYPE"))
        handle.write(rinex_header_line("gnss smartphone-adap libgnss++", "PGM / RUN BY / DATE"))
        comment = "GSDC GPS_L1_CA"
        if self.enable_galileo_e1:
            comment += " + GAL_E1_C_P"
        if self.hatch_window_s is not None:
            comment += f"; Hatch C1C {self.hatch_window_s}s development candidate"
        handle.write(rinex_header_line(comment + "; GPST from ArrivalTimeNanosSinceGpsEpoch", "COMMENT"))
        handle.write(
            rinex_header_line(
                "".join(f"{coordinate:14.4f}" for coordinate in approximate_position),
                "APPROX POSITION XYZ",
            )
        )
        handle.write(rinex_header_line("G    4 C1C L1C D1C S1C", "SYS / # / OBS TYPES"))
        if self.enable_galileo_e1:
            handle.write(rinex_header_line("E    4 C1C L1C D1C S1C", "SYS / # / OBS TYPES"))
        handle.write(rinex_header_line("GPS", "TIME SYSTEM ID"))
        handle.write(rinex_header_line("", "END OF HEADER"))

    def write_epoch(
        self,
        timestamp: int,
        epoch_rows: list[dict[str, str]],
        clock_count: int | None = None,
    ) -> None:
        by_satellite: dict[tuple[str, int], tuple[float, float | None, float, float, str]] = {}
        arrival_times: list[float] = []
        epoch_galileo_prns: set[int] = set()
        if self._hatch is not None and clock_count is None:
            counts = {
                integer(row["HardwareClockDiscontinuityCount"], "HardwareClockDiscontinuityCount", 0)
                for row in epoch_rows
            }
            if len(counts) != 1:
                fail(f"epoch {timestamp}: inconsistent hardware clock discontinuity count")
            clock_count = counts.pop()
        for row in epoch_rows:
            signal = row["SignalType"].strip()
            if signal == GALILEO_E1_SIGNAL and not self.enable_galileo_e1:
                continue
            if signal not in SIGNAL_CONTRACTS:
                continue
            contract = SIGNAL_CONTRACTS[signal]
            constellation = row["ConstellationType"].strip()
            if constellation != contract["constellation_type"]:
                fail(
                    f"epoch {timestamp}: {signal} requires ConstellationType="
                    f"{contract['constellation_type']}, got {constellation!r}"
                )
            frequency = finite(row["CarrierFrequencyHz"], "CarrierFrequencyHz", 0)
            if abs(frequency - float(contract["frequency_hz"])) > 1.0:
                fail(
                    f"epoch {timestamp}: {signal} requires CarrierFrequencyHz="
                    f"{contract['frequency_hz']}, got {frequency}"
                )
            code_type = row["CodeType"].strip()
            if code_type:
                fail(
                    f"epoch {timestamp}: {signal} has unsupported non-empty CodeType "
                    f"{code_type!r}"
                )
            pseudorange_token = row["RawPseudorangeMeters"].strip()
            arrival_token = row["ArrivalTimeNanosSinceGpsEpoch"].strip()
            if not pseudorange_token or not arrival_token:
                continue
            pseudorange = finite(pseudorange_token, "RawPseudorangeMeters", 0)
            arrival_seconds = finite(arrival_token, "ArrivalTimeNanosSinceGpsEpoch", 0) / 1e9
            prn = integer(row["Svid"], "Svid", 0)
            if signal == "GPS_L1_CA":
                prn_min, prn_max = 1, 32
            else:
                prn_min, prn_max = GALILEO_PRN_MIN, GALILEO_PRN_MAX
            if not prn_min <= prn <= prn_max:
                fail(f"epoch {timestamp}: invalid {signal} PRN {prn}")
            rinex_system = str(contract["rinex_system"])
            satellite_key = (rinex_system, prn)
            wavelength = SPEED_OF_LIGHT_MPS / float(contract["frequency_hz"])
            rate = finite(row["PseudorangeRateMetersPerSecond"], "PseudorangeRateMetersPerSecond", 0)
            doppler = -rate / wavelength
            cn0 = finite(row["Cn0DbHz"], "Cn0DbHz", 0)
            adr_token = row["AccumulatedDeltaRangeMeters"].strip()
            adr_state = integer(row["AccumulatedDeltaRangeState"], "AccumulatedDeltaRangeState", 0)
            carrier = None
            if adr_token and adr_state & 1:
                adr_m = finite(adr_token, "AccumulatedDeltaRangeMeters", 0)
                if abs(adr_m) < 1e9:
                    carrier = adr_m / wavelength
            if signal == GALILEO_E1_SIGNAL and self._hatch is not None:
                # ``clock_count`` is guaranteed non-None above when smoothing
                # is enabled; keeping this assertion local makes accidental
                # future call-site omissions fail closed.
                if clock_count is None:
                    fail(f"epoch {timestamp}: missing hardware clock count for Hatch")
                pseudorange = self._hatch.update(
                    key=(rinex_system, prn, signal),
                    timestamp_ms=timestamp,
                    pseudorange_m=pseudorange,
                    adr_token=adr_token,
                    adr_state=adr_state,
                    clock_count=clock_count,
                )
            if satellite_key in by_satellite:
                fail(
                    f"epoch {timestamp}: duplicate {signal} observation for "
                    f"{rinex_system}{prn:02d}"
                )
            by_satellite[satellite_key] = (pseudorange, carrier, doppler, cn0, signal)
            arrival_times.append(arrival_seconds)
            self._signal_rows[signal] = self._signal_rows.get(signal, 0) + 1
            if signal == GALILEO_E1_SIGNAL:
                epoch_galileo_prns.add(prn)
            self._source_rows += 1
        if not by_satellite:
            return
        if max(arrival_times) - min(arrival_times) > 1e-3:
            fail(f"epoch {timestamp}: inconsistent GPS arrival times")
        observations = [(system, prn, *values) for (system, prn), values in sorted(by_satellite.items())]
        stamp = GPS_EPOCH + timedelta(seconds=median(arrival_times))
        second = stamp.second + stamp.microsecond / 1e6
        self._handle.write(
            f"> {stamp.year:04d} {stamp.month:02d} {stamp.day:02d} {stamp.hour:02d} "
            f"{stamp.minute:02d} {second:011.7f}  0{len(observations):3d}\n"
        )
        for system, prn, pseudorange, carrier, doppler, cn0, _signal in observations:
            self._handle.write(
                f"{system}{prn:02d}{rinex_value(pseudorange)}{rinex_value(carrier)}"
                f"{rinex_value(doppler)}{rinex_value(cn0)}\n"
            )
        if epoch_galileo_prns:
            self._galileo_epoch_prns[timestamp] = epoch_galileo_prns
        self._epochs += 1

    def close(self) -> None:
        self._handle.close()

    def summary(self, output_path: Path) -> dict[str, object]:
        if self._epochs == 0:
            fail("no supported observations with raw pseudoranges")
        mapping = {
            signal: list(contract["rinex_observation_types"])
            for signal, contract in SIGNAL_CONTRACTS.items()
            if signal == "GPS_L1_CA" or self.enable_galileo_e1
        }
        return {
            "path": str(output_path),
            "signal_mapping": mapping,
            "source_rows": self._source_rows,
            "signal_rows": dict(sorted(self._signal_rows.items())),
            "epochs": self._epochs,
            "time_system": "GPST",
            "initial_position_source": "device_wls_ecef_first_selected_epoch",
            "file_system": "M" if self.enable_galileo_e1 else "G",
            "galileo_e1_enabled": self.enable_galileo_e1,
            "hatch_carrier_smoothing": (
                self._hatch.summary() if self._hatch is not None else {"enabled": False}
            ),
        }

    @property
    def galileo_epoch_prns(self) -> dict[int, set[int]]:
        return self._galileo_epoch_prns

def validate_galileo_navigation(
    path: Path,
    epoch_prns: dict[int, set[int]],
) -> dict[str, object]:
    """Require a time-covering Galileo broadcast record for every E1 row.

    The native RINEX reader performs the authoritative ephemeris decoding.  This
    small preflight only checks that a mixed nav file contains an E record close
    enough to each selected observation; it intentionally does not synthesize
    ephemerides or guess a missing PRN.
    """

    if not path.is_file():
        fail(f"missing Galileo broadcast navigation: {path}")
    records: dict[int, list[datetime]] = {}
    malformed = 0
    try:
        handle = path.open(encoding="ascii", errors="replace")
    except OSError as exc:
        fail(f"cannot read Galileo broadcast navigation {path}: {exc}")
    with handle:
        for line_number, line in enumerate(handle, start=1):
            if not (len(line) >= 3 and line[0] == "E" and line[1:3].isdigit()):
                continue
            try:
                # RINEX navigation fields are fixed-width; the first clock
                # coefficient starts immediately after the two-digit seconds
                # field, so split() is not valid when that coefficient is
                # negative (for example ``00-1.9E-05``).
                if len(line) < 23:
                    raise ValueError("short Galileo navigation record")
                year = int(line[4:8])
                month = int(line[9:11])
                day = int(line[12:14])
                hour = int(line[15:17])
                minute = int(line[18:20])
                second = float(line[21:23])
                if not math.isfinite(second) or second < 0.0 or second >= 61.0:
                    raise ValueError("invalid navigation seconds")
                whole_second = math.floor(second)
                epoch = datetime(
                    year,
                    month,
                    day,
                    hour,
                    minute,
                    whole_second,
                    tzinfo=timezone.utc,
                ) + timedelta(seconds=second - whole_second)
            except (TypeError, ValueError, OverflowError):
                malformed += 1
                fail(f"line {line_number}: malformed Galileo navigation record")
            prn = int(line[1:3])
            records.setdefault(prn, []).append(epoch)
    if malformed or not records:
        fail(f"Galileo broadcast navigation has no usable E records: {path}")

    observation_rows = sum(len(prns) for prns in epoch_prns.values())
    covered_rows = 0
    nearest_age = 0.0
    missing: list[str] = []
    route_times = list(epoch_prns)
    for timestamp, prns in epoch_prns.items():
        observation_time = datetime.fromtimestamp(timestamp / 1000.0, tz=timezone.utc)
        for prn in sorted(prns):
            candidates = records.get(prn, [])
            if not candidates:
                missing.append(f"E{prn:02d}@{timestamp}")
                continue
            age = min(abs((record - observation_time).total_seconds()) for record in candidates)
            if age > NAVIGATION_MAX_AGE_SECONDS:
                missing.append(f"E{prn:02d}@{timestamp} age={age:.1f}s")
                continue
            covered_rows += 1
            nearest_age = max(nearest_age, age)
    if missing:
        fail(
            "Galileo broadcast navigation does not cover selected observations: "
            + ", ".join(missing[:5])
            + (" ..." if len(missing) > 5 else "")
        )
    return {
        "path": str(path),
        "sha256": sha256(path),
        "system": "Galileo",
        "record_count": sum(len(epochs) for epochs in records.values()),
        "record_prns": sorted(records),
        "observed_prns": sorted({prn for prns in epoch_prns.values() for prn in prns}),
        "route_epochs_with_galileo_e1": len(route_times),
        "route_observation_rows": observation_rows,
        "route_observation_rows_with_nav": covered_rows,
        "route_observation_nav_coverage": covered_rows / observation_rows if observation_rows else 0.0,
        "max_nearest_record_age_s": nearest_age,
        "max_allowed_record_age_s": NAVIGATION_MAX_AGE_SECONDS,
        "source_policy": "native-rinex-nav-required; no missing-PRN synthesis",
    }


def read_truth(path: Path) -> dict[int, tuple[float, float, float]]:
    """Validate truth rows while retaining only the small timestamp index."""

    truth: dict[int, tuple[float, float, float]] = {}
    last_truth_timestamp: int | None = None
    saw_row = False
    with open_csv_reader(path, TRUTH_REQUIRED) as (_, reader):
        for row_number, row in enumerate(reader, start=2):
            saw_row = True
            if row["MessageType"] != "Fix" or row["Provider"] != "GT":
                fail(f"truth row {row_number}: expected MessageType=Fix and Provider=GT")
            timestamp = integer(row["UnixTimeMillis"], "UnixTimeMillis", row_number)
            if last_truth_timestamp is not None and timestamp <= last_truth_timestamp:
                fail(f"truth row {row_number}: timestamps must be strictly increasing")
            last_truth_timestamp = timestamp
            truth[timestamp] = (
                finite(row["LatitudeDegrees"], "LatitudeDegrees", row_number),
                finite(row["LongitudeDegrees"], "LongitudeDegrees", row_number),
                finite(row["AltitudeMeters"], "AltitudeMeters", row_number),
            )
    if not saw_row:
        fail(f"no data rows in {path}")
    return truth


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME"))
    parser.add_argument("--device-gnss", type=Path, required=True)
    parser.add_argument(
        "--ground-truth",
        type=Path,
        help="Independent truth used only for the optional post-adapter score.",
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dataset-id", required=True)
    parser.add_argument("--device-model", required=True)
    parser.add_argument("--source-url", required=True)
    parser.add_argument("--source-terms", required=True)
    parser.add_argument("--role", choices=("development", "holdout", "test"), required=True)
    parser.add_argument("--skip-epochs", type=int, default=0)
    parser.add_argument("--max-epochs", type=int, default=-1)
    parser.add_argument(
        "--experimental-galileo-e1",
        action="store_true",
        help="Development-only candidate: include exactly mapped Galileo E1 observations.",
    )
    parser.add_argument(
        "--experimental-galileo-e1-hatch-window-s",
        type=int,
        choices=HATCH_WINDOW_SECONDS,
        help=(
            "Development-only truth-free Hatch C1C candidate for Galileo E1 "
            "(pre-declared one-Hz windows: 10, 30, or 60 seconds)."
        ),
    )
    parser.add_argument(
        "--broadcast-nav",
        type=Path,
        help="Mixed broadcast navigation required by an experimental signal candidate.",
    )
    parser.add_argument(
        "--allow-missing-truth",
        action="store_true",
        help="Inventory-only mode; accuracy evidence remains unavailable.",
    )
    parser.add_argument(
        "--truth-free",
        action="store_true",
        help=(
            "Development-only pipeline mode: emit normalized observations/RINEX "
            "without opening or hashing ground truth."
        ),
    )
    parser.add_argument(
        "--sealed-holdout-eval-freeze",
        type=Path,
        help=(
            "Explicit one-shot release-freeze authorization for a truth-free "
            "holdout lane; ordinary workflows must not use this option."
        ),
    )
    parser.add_argument(
        "--sealed-test-authorization",
        type=Path,
        help=(
            "Explicit truth-free full-test authorization; the record must pin "
            "the route allowlist and this adapter source hash."
        ),
    )
    parser.add_argument(
        "--sealed-test-authorization-manifest",
        type=Path,
        help="Hash manifest for --sealed-test-authorization.",
    )
    return parser.parse_args()


def _verify_sealed_holdout_eval_freeze(path: Path) -> None:
    """Require the caller to name the immutable pre-holdout freeze record."""

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid sealed holdout freeze record: {exc}")
    if not isinstance(payload, dict) or payload.get("schema_version") not in {
        "smartphone-r5-wls-stability-selector-holdout-freeze.v1",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v2",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v3",
        "smartphone-r5-wls-stability-selector-holdout-freeze.v4",
        "smartphone-r5-gsdc2023-native-fgo-holdout-freeze.v1",
    }:
        fail("sealed holdout freeze record schema is invalid")
    contract = payload.get("holdout_execution_contract")
    if not isinstance(contract, dict) or contract.get("authorized") is not True:
        fail("sealed holdout freeze record does not authorize this lane")
    source_files = payload.get("source_files")
    # Freeze records deliberately key source files by their repository-relative
    # path.  Keep this mapping explicit so a basename-only lookup cannot accept
    # the wrong source file when similarly named fixtures are present.
    adapter_source = (
        source_files.get("apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py")
        if isinstance(source_files, dict)
        else None
    )
    expected = adapter_source.get("sha256") if isinstance(adapter_source, dict) else None
    if expected != sha256(Path(__file__)):
        fail("adapter source hash differs from the sealed holdout freeze")


def _verify_sealed_test_authorization(
    path: Path, manifest_path: Path | None, dataset_id: str
) -> None:
    """Authorize one truth-free test dataset from a content-hashed allowlist."""

    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid sealed test authorization: {exc}")
    if not isinstance(payload, dict) or payload.get("schema_version") != TEST_AUTHORIZATION_SCHEMA:
        fail("sealed test authorization schema is invalid")
    contract = payload.get("test_execution_contract")
    if not isinstance(contract, dict):
        fail("sealed test authorization contract is missing")
    for key, expected in (
        ("authorized", True),
        ("role", "test"),
        ("truth_free_phase", True),
        ("truth_access_forbidden", True),
        ("no_post_test_tuning", True),
    ):
        if contract.get(key) != expected:
            fail(f"sealed test authorization differs for {key}")
    allowlist = contract.get("dataset_allowlist")
    if not isinstance(allowlist, list) or dataset_id not in allowlist:
        fail("test dataset is outside the sealed authorization allowlist")
    source_files = payload.get("source_files")
    adapter_source = (
        source_files.get("apps/commands/benchmarks/gnss_smartphone_gnss_adapter.py")
        if isinstance(source_files, dict)
        else None
    )
    expected_source = (
        adapter_source.get("sha256")
        if isinstance(adapter_source, dict)
        else adapter_source
    )
    if expected_source != sha256(Path(__file__)):
        fail("adapter source hash differs from the sealed test authorization")
    if manifest_path is None:
        fail("sealed test authorization manifest is required")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        fail(f"invalid sealed test authorization manifest: {exc}")
    if not isinstance(manifest, dict) or manifest.get("schema_version") != TEST_AUTHORIZATION_MANIFEST_SCHEMA:
        fail("sealed test authorization manifest schema is invalid")
    record = manifest.get("authorization_record")
    if not isinstance(record, dict) or record.get("sha256") != sha256(path):
        fail("sealed test authorization record hash differs")
    if manifest.get("truth_open_count") != 0 or manifest.get("truth_materialized_count") != 0:
        fail("sealed test authorization records truth access")


def main() -> int:
    args = parse_args()
    sealed_holdout = args.role == "holdout" and args.sealed_holdout_eval_freeze is not None
    sealed_test = args.role == "test" and args.sealed_test_authorization is not None
    if sealed_holdout:
        _verify_sealed_holdout_eval_freeze(args.sealed_holdout_eval_freeze)
    if args.role == "holdout" and args.sealed_test_authorization is not None:
        fail("test authorization is invalid for role=holdout")
    if args.role == "test":
        if not sealed_test:
            fail("role=test requires the sealed test authorization")
        if args.sealed_holdout_eval_freeze is not None:
            fail("holdout freeze is invalid for role=test")
        _verify_sealed_test_authorization(
            args.sealed_test_authorization,
            args.sealed_test_authorization_manifest,
            args.dataset_id,
        )
    if args.role == "development" and (
        args.sealed_holdout_eval_freeze is not None
        or args.sealed_test_authorization is not None
        or args.sealed_test_authorization_manifest is not None
    ):
        fail("sealed authorization is invalid for role=development")
    if args.truth_free:
        if args.role != "development" and not sealed_holdout:
            if not sealed_test:
                fail("--truth-free requires a sealed holdout or test authorization")
        if args.ground_truth is not None:
            fail("--truth-free must not be combined with --ground-truth")
    elif args.ground_truth is None:
        fail("--ground-truth is required unless --truth-free is selected")
    if args.skip_epochs < 0:
        fail("--skip-epochs must be zero or greater")
    if args.max_epochs == 0 or args.max_epochs < -1:
        fail("--max-epochs must be -1 or a positive integer")
    if args.experimental_galileo_e1 and args.role != "development" and not sealed_holdout and not sealed_test:
        fail(
            "development-only: --experimental-galileo-e1 requires "
            "a sealed holdout or test authorization"
        )
    if args.experimental_galileo_e1 and args.broadcast_nav is None:
        fail("--broadcast-nav is required with --experimental-galileo-e1")
    if not args.experimental_galileo_e1 and args.broadcast_nav is not None:
        fail("--broadcast-nav is only valid with --experimental-galileo-e1")
    if args.experimental_galileo_e1_hatch_window_s is not None:
        if not args.experimental_galileo_e1:
            fail(
                "--experimental-galileo-e1-hatch-window-s requires "
                "--experimental-galileo-e1"
            )
        if args.role != "development" and not sealed_holdout and not sealed_test:
            fail(
                "development-only: Hatch smoothing requires "
                "a sealed holdout or test authorization"
            )

    args.output_dir.mkdir(parents=True, exist_ok=True)

    normalized_path = args.output_dir / "observations.csv"
    rinex_path = args.output_dir / "rover.obs"
    receiver_path = args.output_dir / "receiver_wls.csv"
    reference_path = args.output_dir / "reference.csv"
    summary_path = args.output_dir / "summary.json"

    selected_timestamps: list[int] = []
    selected_positions: dict[int, tuple[float, float, float]] = {}
    clock_counts: list[int] = []
    signals: dict[str, int] = {}
    constellations: dict[str, int] = {}
    code_types: dict[str, int] = {}
    carrier_frequencies: dict[str, int] = {}
    unsupported_signal_rows: dict[str, int] = {}
    unmapped_signal_rows = 0
    selected_rows_count = 0
    input_epoch_count = 0
    rinex_summary: dict[str, object] | None = None
    rinex_writer: StreamingRinexWriter | None = None

    try:
        with tempfile.TemporaryDirectory(
            prefix=".smartphone-adapter-", dir=str(args.output_dir)
        ) as temporary_name:
            temporary_dir = Path(temporary_name)
            normalized_tmp = temporary_dir / normalized_path.name
            rinex_tmp = temporary_dir / rinex_path.name
            receiver_tmp = temporary_dir / receiver_path.name
            reference_tmp = temporary_dir / reference_path.name
            summary_tmp = temporary_dir / summary_path.name

            with normalized_tmp.open("w", encoding="utf-8", newline="") as normalized_handle:
                with open_csv_reader(args.device_gnss, DEVICE_REQUIRED) as (
                    fields, device_reader
                ):
                    normalized_writer = csv.DictWriter(
                        normalized_handle, fieldnames=fields, lineterminator="\n"
                    )
                    normalized_writer.writeheader()
                    current_timestamp: int | None = None
                    current_epoch_rows: list[dict[str, str]] = []
                    last_timestamp: int | None = None
                    saw_device_row = False

                    def finish_epoch(
                        timestamp: int, epoch_rows: list[dict[str, str]], epoch_index: int
                    ) -> None:
                        nonlocal rinex_writer
                        nonlocal selected_rows_count
                        nonlocal unmapped_signal_rows
                        selected = epoch_index >= args.skip_epochs and (
                            args.max_epochs < 1
                            or len(selected_timestamps) < args.max_epochs
                        )
                        if not selected:
                            return

                        counts = {
                            integer(
                                row["HardwareClockDiscontinuityCount"],
                                "HardwareClockDiscontinuityCount",
                                0,
                            )
                            for row in epoch_rows
                        }
                        if len(counts) != 1:
                            fail(
                                f"epoch {timestamp}: inconsistent hardware clock discontinuity count"
                            )
                        count = counts.pop()
                        if clock_counts and count < clock_counts[-1]:
                            fail(
                                f"epoch {timestamp}: hardware clock discontinuity count moved backwards"
                            )
                        clock_counts.append(count)

                        positions = []
                        for row in epoch_rows:
                            signal = row["SignalType"].strip()
                            if not signal:
                                signal = "<unmapped>"
                                unmapped_signal_rows += 1
                            signals[signal] = signals.get(signal, 0) + 1
                            code_type = row["CodeType"].strip() or "<empty>"
                            code_types[code_type] = code_types.get(code_type, 0) + 1
                            carrier_token = row["CarrierFrequencyHz"].strip()
                            if carrier_token:
                                carrier_hz = finite(
                                    carrier_token, "CarrierFrequencyHz", 0
                                )
                                carrier_key = f"{carrier_hz:.0f}"
                            else:
                                carrier_key = "<empty>"
                            carrier_frequencies[carrier_key] = (
                                carrier_frequencies.get(carrier_key, 0) + 1
                            )
                            if signal not in SIGNAL_CONTRACTS:
                                unsupported_signal_rows[signal] = (
                                    unsupported_signal_rows.get(signal, 0) + 1
                                )
                            constellation = row["ConstellationType"].strip()
                            constellations[constellation] = constellations.get(constellation, 0) + 1
                            positions.append(
                                tuple(
                                    finite(row[field], field, 0)
                                    for field in (
                                        "WlsPositionXEcefMeters",
                                        "WlsPositionYEcefMeters",
                                        "WlsPositionZEcefMeters",
                                    )
                                )
                            )
                        for axis in range(3):
                            if max(position[axis] for position in positions) - min(
                                position[axis] for position in positions
                            ) > 1e-3:
                                fail(f"epoch {timestamp}: inconsistent WLS ECEF position")

                        selected_timestamps.append(timestamp)
                        selected_positions[timestamp] = positions[0]
                        selected_rows_count += len(epoch_rows)
                        normalized_writer.writerows(epoch_rows)
                        if rinex_writer is None:
                            approximate_position = tuple(
                                finite(epoch_rows[0][field], field, 0)
                                for field in (
                                    "WlsPositionXEcefMeters",
                                    "WlsPositionYEcefMeters",
                                    "WlsPositionZEcefMeters",
                                )
                            )
                            rinex_writer = StreamingRinexWriter(
                                rinex_tmp,
                                approximate_position,
                                enable_galileo_e1=args.experimental_galileo_e1,
                                hatch_window_s=args.experimental_galileo_e1_hatch_window_s,
                            )
                        rinex_writer.write_epoch(timestamp, epoch_rows, count)

                    for row_number, row in enumerate(device_reader, start=2):
                        saw_device_row = True
                        if row["MessageType"] != "Raw":
                            fail(f"row {row_number}: MessageType must be Raw")
                        timestamp = integer(row["utcTimeMillis"], "utcTimeMillis", row_number)
                        if last_timestamp is not None and timestamp < last_timestamp:
                            fail(f"row {row_number}: utcTimeMillis moved backwards")
                        last_timestamp = timestamp
                        if current_timestamp is None:
                            current_timestamp = timestamp
                        elif timestamp != current_timestamp:
                            finish_epoch(
                                current_timestamp,
                                current_epoch_rows,
                                input_epoch_count,
                            )
                            input_epoch_count += 1
                            current_timestamp = timestamp
                            current_epoch_rows = []
                        current_epoch_rows.append(row)

                    if not saw_device_row:
                        fail(f"no data rows in {args.device_gnss}")
                    if current_timestamp is not None:
                        finish_epoch(
                            current_timestamp,
                            current_epoch_rows,
                            input_epoch_count,
                        )
                        input_epoch_count += 1

            if args.skip_epochs >= input_epoch_count:
                fail("--skip-epochs removes every input epoch")
            if rinex_writer is None:
                fail("no supported GPS_L1_CA observations with raw pseudoranges")
            rinex_writer.close()
            rinex_summary = rinex_writer.summary(rinex_path)
            if args.experimental_galileo_e1:
                if not rinex_writer.galileo_epoch_prns:
                    fail("--experimental-galileo-e1 found no Galileo E1 observations")
                navigation_summary = validate_galileo_navigation(
                    args.broadcast_nav,
                    rinex_writer.galileo_epoch_prns,
                )
            else:
                navigation_summary = None

            truth: dict[int, tuple[float, float, float]] = {}
            missing_truth: list[int] = []
            if not args.truth_free:
                # The truth file is intentionally opened only in the scored
                # adapter mode.  ``--truth-free`` must never touch this path.
                truth = read_truth(args.ground_truth)
                missing_truth = [
                    timestamp
                    for timestamp in selected_timestamps
                    if timestamp not in truth
                ]
                if missing_truth and not args.allow_missing_truth:
                    fail(f"missing truth for {len(missing_truth)} selected epochs")

            errors_h: list[float] = []
            errors_v: list[float] = []
            with receiver_tmp.open(
                "w", encoding="utf-8", newline=""
            ) as receiver_handle, reference_tmp.open(
                "w", encoding="utf-8", newline=""
            ) as reference_handle:
                receiver_writer = csv.writer(receiver_handle, lineterminator="\n")
                reference_writer = csv.writer(reference_handle, lineterminator="\n")
                receiver_writer.writerow(
                    (
                        "utc_time_millis",
                        "latitude_deg",
                        "longitude_deg",
                        "height_m",
                        "source",
                    )
                )
                reference_writer.writerow(
                    (
                        "utc_time_millis",
                        "latitude_deg",
                        "longitude_deg",
                        "height_m",
                        "source",
                    )
                )
                for timestamp in selected_timestamps:
                    lat, lon, height = ecef_to_geodetic(*selected_positions[timestamp])
                    receiver_writer.writerow(
                        (
                            timestamp,
                            f"{lat:.12f}",
                            f"{lon:.12f}",
                            f"{height:.4f}",
                            "device_wls_ecef",
                        )
                    )
                    if timestamp in truth:
                        truth_lat, truth_lon, truth_height = truth[timestamp]
                        reference_writer.writerow(
                            (
                                timestamp,
                                truth_lat,
                                truth_lon,
                                truth_height,
                                "independent_ground_truth",
                            )
                        )
                        errors_h.append(
                            horizontal_error_m(lat, lon, truth_lat, truth_lon)
                        )
                        errors_v.append(abs(height - truth_height))

            transitions = sum(
                current != previous
                for previous, current in zip(clock_counts, clock_counts[1:])
            )
            supported_signal_policy = (
                ["GPS_L1_CA", GALILEO_E1_SIGNAL]
                if args.experimental_galileo_e1
                else ["GPS_L1_CA"]
            )
            primary_candidate = {
                "name": "galileo-e1",
                "enabled": args.experimental_galileo_e1,
                "role_policy": "development-only; holdout untouched",
                "source_signal": GALILEO_E1_SIGNAL,
                "constellation_type": "6",
                "carrier_frequency_hz": GALILEO_E1_HZ,
                "code_type_policy": "empty-only (source CodeType is empty)",
                "rinex_system": "E",
                "rinex_observation_types": ["C1C", "L1C", "D1C", "S1C"],
                "hatch_carrier_smoothing": {
                    "enabled": args.experimental_galileo_e1_hatch_window_s is not None,
                    "window_seconds": args.experimental_galileo_e1_hatch_window_s,
                    "development_only": True,
                },
            }
            summary = {
                "schema_version": SCHEMA_VERSION,
                "decision": (
                    "truth-free-pipeline"
                    if args.truth_free
                    else ("adapter-pass" if not missing_truth else "inventory-only")
                ),
                "truth_free": args.truth_free,
                "dataset": {
                    "id": args.dataset_id,
                    "role": args.role,
                    "device_model": args.device_model,
                    "source_url": args.source_url,
                    "source_terms": args.source_terms,
                },
                "inputs": {
                    "device_gnss": {
                        "path": str(args.device_gnss),
                        "sha256": sha256(args.device_gnss),
                    },
                    "ground_truth": (
                        {
                            "path": str(args.ground_truth),
                            "sha256": sha256(args.ground_truth),
                        }
                        if not args.truth_free
                        else None
                    ),
                },
                "observations": {
                    "rows": selected_rows_count,
                    "epochs": len(selected_timestamps),
                    "input_epochs": input_epoch_count,
                    "explicitly_skipped_epochs": args.skip_epochs,
                    "first_utc_time_millis": selected_timestamps[0],
                    "last_utc_time_millis": selected_timestamps[-1],
                    "duration_seconds": (
                        selected_timestamps[-1] - selected_timestamps[0]
                    )
                    / 1000.0,
                    "signal_rows": dict(sorted(signals.items())),
                    "unmapped_signal_rows": unmapped_signal_rows,
                    "unmapped_signal_policy": "preserved-but-excluded-from-solver-input",
                    "constellation_rows": dict(sorted(constellations.items())),
                    "code_type_rows": dict(sorted(code_types.items())),
                    "carrier_frequency_hz_rows": dict(
                        sorted(carrier_frequencies.items())
                    ),
                    "unsupported_signal_rows": dict(
                        sorted(unsupported_signal_rows.items())
                    ),
                    "supported_signal_policy": supported_signal_policy,
                    "hardware_clock_discontinuity_transitions": transitions,
                    "hardware_clock_discontinuity_counts": sorted(set(clock_counts)),
                },
                "truth": {
                    "matched_epochs": len(errors_h),
                    "missing_epochs": len(missing_truth),
                    "coverage_ratio": (
                        len(errors_h) / len(selected_timestamps)
                        if not args.truth_free
                        else None
                    ),
                    "independent": not args.truth_free,
                    "used": not args.truth_free,
                },
                "device_wls_score": {
                    "horizontal_median_m": median(errors_h) if errors_h else None,
                    "horizontal_p95_m": percentile(errors_h, 0.95),
                    "vertical_median_abs_m": median(errors_v) if errors_v else None,
                    "accuracy_claim": "baseline-only",
                },
                "native_observation_adapter": rinex_summary,
                "experimental_signal_candidate": primary_candidate,
                "navigation": navigation_summary,
                "artifacts": {
                    "normalized_observations": str(normalized_path),
                    "receiver_wls": str(receiver_path),
                    "reference": str(reference_path),
                    "rinex_observations": str(rinex_path),
                },
            }
            summary_tmp.write_text(
                json.dumps(summary, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            for temporary_path, final_path in (
                (normalized_tmp, normalized_path),
                (rinex_tmp, rinex_path),
                (receiver_tmp, receiver_path),
                (reference_tmp, reference_path),
                (summary_tmp, summary_path),
            ):
                os.replace(temporary_path, final_path)
    finally:
        if rinex_writer is not None:
            rinex_writer.close()

    print(f"Smartphone GNSS adapter complete: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
