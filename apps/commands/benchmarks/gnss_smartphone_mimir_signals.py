#!/usr/bin/env python3
"""Multi-signal contract for the Mimir (Android GnssMeasurement) adapter.

Mimir ``Raw.csv`` has neither ``CodeType`` nor ``SignalType``.  The signal is
therefore *inferred* from ``ConstellationType`` + ``CarrierFrequencyHz``.  The
table below is the complete, closed set of signals the adapter will emit; a
row whose constellation/carrier is not in the table is accounted as rejected
(never silently dropped) with a reason.

Inference limits (documented, not verifiable from the data):

* the tracking attribute of a RINEX code (``C5Q`` versus ``C5I``/``C5X``) is
  not logged by Mimir, so L5-class codes use the pilot/neutral attribute the
  Pixel 7 is known to track (GPS L5-Q, Galileo E5a-Q) or ``X`` (BDS B2a);
  the native reader keys only on the band digit;
* GLONASS G1 is FDMA: the channel ``k`` is derived from the carrier
  (``1602 MHz + k * 562.5 kHz``) and must match the broadcast navigation file.

Time-of-transmission semantics follow the Android ``ReceivedSvTimeNanos``
contract: GPS/Galileo/QZSS are GPST-aligned time of week, BeiDou is BDT
(GPST - 14 s) time of week, GLONASS is time of day in GLONASS time
(UTC + 3 h).
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
import math
from pathlib import Path
from statistics import median

from gnss_smartphone_gnss_adapter import (
    GPS_EPOCH,
    SPEED_OF_LIGHT_MPS,
    HatchSmoother,
    fail,
    rinex_header_line,
    rinex_value,
)

WEEK_NS = 604_800 * 10**9
DAY_NS = 86_400 * 10**9
BDT_MINUS_GPST_S = -14
GLONASS_UTC_PLUS_3H_S = 3 * 3600
GPS_UTC_LEAP_SECONDS = 18  # 2017-01-01 .. present; valid for the 2024 data

FREQ_TOLERANCE_HZ = 1000.0  # Android reports e.g. 1575420030 for L1 (+30 Hz)
GLONASS_G1_BASE_HZ = 1_602_000_000.0
GLONASS_G1_STEP_HZ = 562_500.0
GLONASS_FCN_MIN, GLONASS_FCN_MAX = -7, 6

# Android GnssMeasurement.STATE_* bits (only those used by the usability rules).
STATE_CODE_LOCK = 1
STATE_TOW_DECODED = 8
STATE_MSEC_AMBIGUOUS = 16
STATE_GLO_TOD_DECODED = 1 << 7
STATE_GAL_E1BC_CODE_LOCK = 1 << 10
STATE_TOW_KNOWN = 1 << 14
STATE_GLO_TOD_KNOWN = 1 << 15

NAV_MAX_AGE_S = {"G": 4 * 3600, "E": 4 * 3600, "C": 4 * 3600, "J": 4 * 3600, "R": 1800}


@dataclass(frozen=True)
class SignalSpec:
    name: str  # stable label written to observations.csv
    constellation: str  # Android ConstellationType
    rinex_system: str
    nominal_hz: float | None  # None -> GLONASS FDMA
    obs_codes: tuple[str, str, str, str]  # (code, phase, doppler, snr)
    prn_min: int
    prn_max: int
    svid_offset: int = 0  # prn = svid - svid_offset (QZSS: 192)
    note: str = ""


def _codes(tag: str) -> tuple[str, str, str, str]:
    return (f"C{tag}", f"L{tag}", f"D{tag}", f"S{tag}")


SIGNALS: tuple[SignalSpec, ...] = (
    SignalSpec("GPS_L1_CA", "1", "G", 1_575_420_000.0, _codes("1C"), 1, 32),
    SignalSpec("GPS_L5_Q", "1", "G", 1_176_450_000.0, _codes("5Q"), 1, 32,
               note="tracking attribute Q inferred (no CodeType in Mimir)"),
    SignalSpec("GAL_E1_C_P", "6", "E", 1_575_420_000.0, _codes("1C"), 1, 36),
    SignalSpec("GAL_E5A_Q", "6", "E", 1_176_450_000.0, _codes("5Q"), 1, 36,
               note="tracking attribute Q inferred (no CodeType in Mimir)"),
    SignalSpec("GLO_G1_CA", "3", "R", None, _codes("1C"), 1, 27),
    SignalSpec("BDS_B1I", "5", "C", 1_561_098_000.0, _codes("2I"), 1, 63),
    SignalSpec("BDS_B2A", "5", "C", 1_176_450_000.0, _codes("5X"), 1, 63,
               note="tracking attribute X (data+pilot) used; not logged by Mimir"),
    SignalSpec("BDS_B1C", "5", "C", 1_575_420_000.0, _codes("1X"), 1, 63,
               note="supported by table; absent from the Pixel 7 Nantes data"),
    SignalSpec("QZS_L1_CA", "4", "J", 1_575_420_000.0, _codes("1C"), 1, 10, svid_offset=192,
               note="supported by table; absent from the Pixel 7 Nantes data"),
    SignalSpec("QZS_L5", "4", "J", 1_176_450_000.0, _codes("5X"), 1, 10, svid_offset=192,
               note="supported by table; absent from the Pixel 7 Nantes data"),
)
SIGNAL_BY_NAME = {spec.name: spec for spec in SIGNALS}
SUPPORTED_CONSTELLATIONS = {spec.constellation for spec in SIGNALS}
CONSTELLATION_NAMES = {
    "1": "GPS", "2": "SBAS", "3": "GLONASS", "4": "QZSS", "5": "BeiDou",
    "6": "Galileo", "7": "NavIC",
}
HATCH_SIGNAL = "GAL_E1_C_P"  # the R5 promoted lane smooths Galileo E1 only

MULTI_DISPOSITIONS = (
    "used",
    "unsupported_constellation",
    "unsupported_frequency",
    "signal_not_enabled",
    "invalid_svid",
    "glonass_fcn_conflict",
    "state_not_usable",
    "uncertain_received_sv_time",
    "implausible_travel_time",
    "no_range_fields",
    "no_navigation",
    "excluded_by_max_epochs",
)


def glonass_fcn_from_carrier(carrier_hz: float) -> int | None:
    k = round((carrier_hz - GLONASS_G1_BASE_HZ) / GLONASS_G1_STEP_HZ)
    if not GLONASS_FCN_MIN <= k <= GLONASS_FCN_MAX:
        return None
    if abs(carrier_hz - (GLONASS_G1_BASE_HZ + k * GLONASS_G1_STEP_HZ)) > FREQ_TOLERANCE_HZ:
        return None
    return k


def classify_multi(
    constellation: str, carrier_hz: float | None
) -> tuple[SignalSpec | None, int | None, str]:
    """Return ``(spec, glonass_fcn, reason)``; ``spec`` is None when rejected."""

    if constellation not in SUPPORTED_CONSTELLATIONS:
        return None, None, "unsupported_constellation"
    if carrier_hz is None:
        return None, None, "unsupported_frequency"
    for spec in SIGNALS:
        if spec.constellation != constellation:
            continue
        if spec.nominal_hz is None:
            fcn = glonass_fcn_from_carrier(carrier_hz)
            if fcn is not None:
                return spec, fcn, ""
        elif abs(carrier_hz - spec.nominal_hz) <= FREQ_TOLERANCE_HZ:
            return spec, None, ""
    return None, None, "unsupported_frequency"


def prn_from_svid(spec: SignalSpec, svid: int) -> int | None:
    prn = svid - spec.svid_offset
    return prn if spec.prn_min <= prn <= spec.prn_max else None


def state_usable_multi(constellation: str, state: int) -> bool:
    """Whether ReceivedSvTimeNanos is an unambiguous time of transmission."""

    if state & STATE_MSEC_AMBIGUOUS:
        return False
    if constellation == "3":
        return bool(state & STATE_CODE_LOCK) and bool(state & (STATE_GLO_TOD_DECODED | STATE_GLO_TOD_KNOWN))
    tow_known = bool(state & (STATE_TOW_DECODED | STATE_TOW_KNOWN))
    if constellation == "6":
        return bool(state & (STATE_CODE_LOCK | STATE_GAL_E1BC_CODE_LOCK)) and tow_known
    if constellation in ("1", "4", "5"):
        return bool(state & STATE_CODE_LOCK) and tow_known
    return False


def travel_time_s(constellation: str, whole_ns: int, frac_ns: float, tx_ns: float) -> float:
    """Signal travel time with the constellation-specific time base."""

    week = math.floor(whole_ns / WEEK_NS)
    tow_rx_ns = (whole_ns - week * WEEK_NS) + frac_ns
    if constellation == "3":
        rx_ns = (tow_rx_ns + (GLONASS_UTC_PLUS_3H_S - GPS_UTC_LEAP_SECONDS) * 1e9) % DAY_NS
        period = DAY_NS
    else:
        rx_ns = tow_rx_ns + (BDT_MINUS_GPST_S * 1e9 if constellation == "5" else 0.0)
        period = WEEK_NS
    travel_ns = rx_ns - tx_ns
    if travel_ns > period / 2:
        travel_ns -= period
    elif travel_ns < -period / 2:
        travel_ns += period
    return travel_ns * 1e-9


# ---------------------------------------------------------------- navigation


class NavigationIndex:
    """Per ``(system, prn)`` broadcast record epochs (+ GLONASS channels)."""

    def __init__(self) -> None:
        self.records: dict[tuple[str, int], list[datetime]] = {}
        self.glonass_channels: dict[int, set[int]] = {}

    @classmethod
    def load(cls, path: Path) -> "NavigationIndex":
        index = cls()
        with path.open(encoding="ascii", errors="replace") as handle:
            lines = handle.readlines()
        i = 0
        while i < len(lines):
            line = lines[i]
            if len(line) >= 23 and line[0] in "GRECJ" and line[1:3].isdigit() and line[3] == " ":
                try:
                    second = float(line[21:23])
                    epoch = datetime(
                        int(line[4:8]), int(line[9:11]), int(line[12:14]),
                        int(line[15:17]), int(line[18:20]), int(second), tzinfo=timezone.utc,
                    )
                except (ValueError, OverflowError):
                    i += 1
                    continue
                system, prn = line[0], int(line[1:3])
                index.records.setdefault((system, prn), []).append(epoch)
                if system == "R" and i + 2 < len(lines):
                    try:
                        index.glonass_channels.setdefault(prn, set()).add(
                            int(round(float(lines[i + 2][61:80].replace("D", "E"))))
                        )
                    except ValueError:
                        pass
            i += 1
        if not index.records:
            raise SystemExit(f"navigation file has no usable records: {path}")
        return index

    def covers(self, system: str, prn: int, gpst: datetime) -> tuple[bool, float | None]:
        epochs = self.records.get((system, prn))
        if not epochs:
            return False, None
        age = min(abs((e - gpst).total_seconds()) for e in epochs)
        return age <= NAV_MAX_AGE_S[system], age


# --------------------------------------------------------------------- RINEX


def rinex_obs_type_lines(specs: tuple[SignalSpec, ...]) -> list[tuple[str, str]]:
    """``(content, label)`` SYS / # / OBS TYPES lines for the enabled specs."""

    by_system: dict[str, list[str]] = {}
    for spec in specs:
        by_system.setdefault(spec.rinex_system, []).extend(spec.obs_codes)
    lines = []
    for system in sorted(by_system):
        types = by_system[system]
        if len(types) > 13:
            raise SystemExit("too many observation types for one header line")
        lines.append((f"{system}  {len(types):3d} " + " ".join(types), "SYS / # / OBS TYPES"))
    return lines


def glonass_slot_frq_lines(channels: dict[int, int]) -> list[tuple[str, str]]:
    if not channels:
        return []
    items = sorted(channels.items())
    lines = []
    for first in range(0, len(items), 8):
        chunk = items[first:first + 8]
        prefix = f"{len(items):3d} " if first == 0 else "    "
        lines.append((prefix + "".join(f"R{slot:02d} {k:2d} " for slot, k in chunk),
                      "GLONASS SLOT / FRQ #"))
    return lines


def gpst_to_datetime(seconds_since_gps_epoch: float) -> datetime:
    return datetime(1980, 1, 6, tzinfo=timezone.utc) + timedelta(seconds=seconds_since_gps_epoch)


@dataclass
class ObservationRecord:
    """One accepted measurement handed to :class:`MultiSignalRinexWriter`."""

    spec: SignalSpec
    prn: int
    pseudorange_m: float
    arrival_ns: float
    carrier_hz: float
    doppler_rate_mps: float
    cn0_dbhz: float
    adr_state: int
    adr_token: str


class MultiSignalRinexWriter:
    """RINEX 3.04 writer for several signals per satellite.

    Same epoch/line conventions as the R5 ``StreamingRinexWriter`` (GPST epoch
    from the median arrival time, Doppler = -range_rate / wavelength, carrier
    from valid ADR only), but several signals of one satellite share a line.
    The R5 Hatch smoother is reused unchanged and applied to Galileo E1 only.
    """

    def __init__(
        self,
        path: Path,
        approximate_position: tuple[float, float, float],
        specs: tuple[SignalSpec, ...],
        glonass_channels: dict[int, int],
        hatch_window_s: int | None = None,
    ) -> None:
        self.specs = specs
        self.hatch_window_s = hatch_window_s
        self._hatch = HatchSmoother(hatch_window_s) if hatch_window_s is not None else None
        self._layout: dict[str, list[SignalSpec]] = {}
        for spec in specs:
            self._layout.setdefault(spec.rinex_system, []).append(spec)
        self._handle = path.open("w", encoding="ascii", newline="")
        self._epochs = 0
        self._signal_rows: dict[str, int] = {}
        self._write_header(approximate_position, glonass_channels)

    def _write_header(self, approx, glonass_channels) -> None:
        h = self._handle
        h.write(rinex_header_line("     3.04           OBSERVATION DATA    M", "RINEX VERSION / TYPE"))
        h.write(rinex_header_line("gnss smartphone-mimir libgnss++", "PGM / RUN BY / DATE"))
        h.write(rinex_header_line("Mimir multi-signal; signal table in adapter summary", "COMMENT"))
        if self.hatch_window_s is not None:
            h.write(rinex_header_line(
                f"Hatch C1C {self.hatch_window_s}s on {HATCH_SIGNAL} (R5 lane)", "COMMENT"))
        h.write(rinex_header_line("".join(f"{c:14.4f}" for c in approx), "APPROX POSITION XYZ"))
        for content, label in rinex_obs_type_lines(self.specs):
            h.write(rinex_header_line(content, label))
        for content, label in glonass_slot_frq_lines(glonass_channels):
            h.write(rinex_header_line(content, label))
        h.write(rinex_header_line("GPS", "TIME SYSTEM ID"))
        h.write(rinex_header_line("", "END OF HEADER"))

    def write_epoch(self, timestamp_ms: int, records: list[ObservationRecord], clock_count: int) -> None:
        by_sat: dict[tuple[str, int], dict[str, tuple]] = {}
        arrivals = []
        for rec in records:
            spec = rec.spec
            wavelength = SPEED_OF_LIGHT_MPS / rec.carrier_hz
            carrier = None
            if rec.adr_token and rec.adr_state & 1:
                adr_m = float(rec.adr_token)
                if math.isfinite(adr_m) and abs(adr_m) < 1e9:
                    carrier = adr_m / wavelength
            pseudorange = rec.pseudorange_m
            if spec.name == HATCH_SIGNAL and self._hatch is not None:
                pseudorange = self._hatch.update(
                    key=(spec.rinex_system, rec.prn, spec.name),
                    timestamp_ms=timestamp_ms,
                    pseudorange_m=pseudorange,
                    adr_token=rec.adr_token,
                    adr_state=rec.adr_state,
                    clock_count=clock_count,
                )
            sat = by_sat.setdefault((spec.rinex_system, rec.prn), {})
            if spec.name in sat:
                fail(f"epoch {timestamp_ms}: duplicate {spec.name} observation for "
                     f"{spec.rinex_system}{rec.prn:02d}")
            sat[spec.name] = (pseudorange, carrier, -rec.doppler_rate_mps / wavelength, rec.cn0_dbhz)
            arrivals.append(rec.arrival_ns / 1e9)
            self._signal_rows[spec.name] = self._signal_rows.get(spec.name, 0) + 1
        if not by_sat:
            return
        if max(arrivals) - min(arrivals) > 1e-3:
            fail(f"epoch {timestamp_ms}: inconsistent GPS arrival times "
                 f"({(max(arrivals) - min(arrivals)) * 1e3:.3f} ms)")
        stamp = GPS_EPOCH + timedelta(seconds=median(arrivals))
        second = stamp.second + stamp.microsecond / 1e6
        self._handle.write(
            f"> {stamp.year:04d} {stamp.month:02d} {stamp.day:02d} {stamp.hour:02d} "
            f"{stamp.minute:02d} {second:011.7f}  0{len(by_sat):3d}\n"
        )
        for (system, prn), signals in sorted(by_sat.items()):
            fields = []
            for spec in self._layout[system]:
                values = signals.get(spec.name)
                fields.extend(values if values is not None else (None, None, None, None))
            self._handle.write(
                f"{system}{prn:02d}" + "".join(rinex_value(v) for v in fields) + "\n"
            )
        self._epochs += 1

    def close(self) -> None:
        self._handle.close()

    def summary(self, output_path: Path) -> dict[str, object]:
        if self._epochs == 0:
            fail("no supported observations with raw pseudoranges")
        return {
            "path": str(output_path),
            "writer": "MultiSignalRinexWriter (mimir multi-signal)",
            "signal_mapping": {s.name: list(s.obs_codes) for s in self.specs},
            "signal_notes": {s.name: s.note for s in self.specs if s.note},
            "signal_rows": dict(sorted(self._signal_rows.items())),
            "source_rows": sum(self._signal_rows.values()),
            "epochs": self._epochs,
            "time_system": "GPST",
            "hatch_carrier_smoothing": (
                self._hatch.summary() if self._hatch is not None else {"enabled": False}
            ),
        }
