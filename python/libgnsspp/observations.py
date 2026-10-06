"""Satellite observation diagnostics from public corrected-measurement epochs.

Code residuals remove a fitted clock offset in each native clock group. They
are diagnostics at the SPP position, not the solver's admitted-row residuals.
Carrier discontinuities use source LLI and trapezoidal Doppler integration;
they are indications, not proof that an ambiguity jumped by an integer.
"""
from __future__ import annotations

from collections import Counter, defaultdict
import math
import statistics
from typing import Any, Iterable

C = 299792458.0


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (ValueError, TypeError, OverflowError):
        return None
    return number if math.isfinite(number) else None


def quantile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    return ordered[lo] + (index - lo) * (ordered[hi] - ordered[lo])


def analyze_epochs(epochs: Iterable[tuple[Any, Iterable[Any]]], *,
                   slip_threshold_cycles: float = 10.0, max_gap_s: float = 2.0,
                   min_clock_witnesses: int = 4) -> tuple[list[dict], dict]:
    """Return JSON-safe observation records and a summary, without using truth.

    ``epochs`` is the return value of ``libgnsspp.preprocess_spp_file``.
    A clock-discontinuity candidate needs at least four comparable satellites
    and agreement by at least 75 percent. Both raw and adjusted phase/Doppler
    evidence remain in the output. A signal or tracking-code change starts a
    new arc, as do gaps and missing carrier measurements.
    """
    if not math.isfinite(slip_threshold_cycles) or slip_threshold_cycles <= 0:
        raise ValueError("slip_threshold_cycles must be finite and positive")
    if not math.isfinite(max_gap_s) or max_gap_s <= 0:
        raise ValueError("max_gap_s must be finite and positive")
    if type(min_clock_witnesses) is not int or min_clock_witnesses < 4:
        raise ValueError("min_clock_witnesses must be at least four")
    records: list[dict] = []
    previous: dict[tuple, dict] = {}
    previous_tracks: dict[tuple, set[str]] = {}
    previous_time = -math.inf
    epoch_count = valid_count = 0
    for solution, measurements in epochs:
        week, tow = int(solution.time.week), finite(solution.time.tow)
        if tow is None or not 0 <= tow < 604800:
            raise ValueError("epoch time must have a finite normalized GPS TOW")
        stamp = week * 604800.0 + tow
        if stamp <= previous_time:
            raise ValueError("observation epochs must be strictly increasing")
        previous_time = stamp
        epoch_count += 1
        position = [finite(v) for v in solution.position_ecef_m]
        valid = bool(solution.is_valid()) and all(v is not None for v in position)
        valid_count += int(valid)
        current: list[dict] = []
        keys = set()
        clock_rows: dict[int, list[dict]] = defaultdict(list)
        measurements = list(measurements)
        epoch_tracks: dict[tuple, set[str]] = defaultdict(set)
        for measurement in measurements:
            epoch_tracks[(str(measurement.satellite_id), int(measurement.signal_id))].add(
                str(measurement.carrier_observation_type))
        for pair, tracks in epoch_tracks.items():
            if pair in previous_tracks and tracks != previous_tracks[pair]:
                for key in [key for key in previous if key[:2] == pair]:
                    previous.pop(key)
            previous_tracks[pair] = tracks
        for measurement in measurements:
            satellite = str(measurement.satellite_id)
            signal = int(measurement.signal_id)
            tracking = str(measurement.carrier_observation_type)
            key = (satellite, signal, tracking)
            if key in keys:
                raise ValueError(f"duplicate satellite/signal/tracking row at {week}:{tow}: {key}")
            keys.add(key)
            frequency = finite(measurement.carrier_frequency_hz)
            wavelength = C / frequency if frequency is not None and frequency > 0 else None
            phase, doppler = finite(measurement.carrier_phase), finite(measurement.doppler)
            snr = finite(measurement.snr)
            elevation = finite(measurement.elevation)
            satellite_velocity = [finite(v) for v in measurement.satellite_velocity]
            row = {"week": week, "tow_s": tow, "gps_seconds": stamp,
                   "satellite_id": satellite, "signal_id": signal,
                   "carrier_observation_type": tracking, "clock_group": int(measurement.clock_group),
                   "carrier_frequency_hz": frequency, "wavelength_m": wavelength,
                   "ionosphere_free_code": bool(measurement.ionosphere_free),
                   "snr_dbhz": snr if snr is not None and snr > 0 else None,
                   "elevation_deg": math.degrees(elevation) if elevation is not None else None,
                   "corrected_pseudorange_m": finite(measurement.corrected_pseudorange),
                   "carrier_phase_cycles": phase, "doppler_hz": doppler,
                   "satellite_speed_mps": math.hypot(*satellite_velocity) if all(v is not None for v in satellite_velocity) else None,
                   "satellite_clock_drift_sps": finite(measurement.satellite_clock_drift),
                   "solution_valid": valid, "code_minus_range_m": None,
                   "fitted_clock_bias_m": None, "clock_group_witnesses": 0,
                   "clock_removed_code_residual_m": None,
                   "normalized_code_residual": None,
                   "phase_doppler_raw_cycles": None, "phase_doppler_adjusted_cycles": None,
                   "common_clock_step_m": None, "clock_step_candidate": False,
                   "loss_of_lock_indicator": int(measurement.loss_of_lock_indicator),
                   "source_loss_of_lock": bool(measurement.source_loss_of_lock),
                   "continuity_assessed": False, "slip_suspect": False, "reason_codes": []}
            satellite_ecef = [finite(v) for v in measurement.satellite_ecef]
            weight, variance = finite(measurement.weight), finite(measurement.variance)
            row["weight_inv_m2"], row["variance_m2"] = weight, variance
            if valid and all(v is not None for v in satellite_ecef) and row["corrected_pseudorange_m"] is not None:
                row["code_minus_range_m"] = row["corrected_pseudorange_m"] - math.dist(position, satellite_ecef)
                if weight is not None and weight > 0:
                    clock_rows[row["clock_group"]].append(row)
            old = previous.get(key)
            if phase is None:
                row["reason_codes"].append("carrier_unavailable")
                previous.pop(key, None)
            else:
                if row["source_loss_of_lock"] or row["loss_of_lock_indicator"] & 1:
                    row["reason_codes"].append("source_loss_of_lock")
                    row["slip_suspect"] = True
                if row["loss_of_lock_indicator"] & 2:
                    row["reason_codes"].append("half_cycle_ambiguity")
                    row["slip_suspect"] = True
                if old is None:
                    row["reason_codes"].append("arc_start")
                elif stamp - old["gps_seconds"] > max_gap_s:
                    row["reason_codes"].append("gap_reset")
                elif doppler is None or old["doppler_hz"] is None:
                    row["reason_codes"].append("doppler_unavailable")
                elif wavelength is None or old["wavelength_m"] != wavelength:
                    row["reason_codes"].append("frequency_unavailable_or_changed")
                else:
                    dt = stamp - old["gps_seconds"]
                    row["phase_doppler_raw_cycles"] = phase - old["carrier_phase_cycles"] + 0.5 * (doppler + old["doppler_hz"]) * dt
                    row["phase_doppler_adjusted_cycles"] = row["phase_doppler_raw_cycles"]
                    row["continuity_assessed"] = True
                previous[key] = row
            current.append(row)
        for group in clock_rows.values():
            for row in group:
                row["clock_group_witnesses"] = len(group)
            if len(group) < 2:
                group[0]["reason_codes"].append("clock_group_too_small")
                continue
            denominator = math.fsum(row["weight_inv_m2"] for row in group)
            # Subtract a nearby offset before summing to preserve small residuals
            # in the presence of a large receiver clock bias.
            origin = group[0]["code_minus_range_m"]
            clock = origin + math.fsum(row["weight_inv_m2"] * (row["code_minus_range_m"] - origin)
                                       for row in group) / denominator
            for row in group:
                residual = row["code_minus_range_m"] - clock
                row["fitted_clock_bias_m"] = clock
                row["clock_removed_code_residual_m"] = residual
                if row["variance_m2"] is not None and row["variance_m2"] > 0:
                    row["normalized_code_residual"] = residual / math.sqrt(row["variance_m2"])
        phase_groups: dict[int, list[dict]] = defaultdict(list)
        for row in current:
            if row["continuity_assessed"]:
                phase_groups[row["clock_group"]].append(row)
        for group in phase_groups.values():
            step = statistics.median(row["phase_doppler_raw_cycles"] * row["wavelength_m"] for row in group)
            agrees = [abs(row["phase_doppler_raw_cycles"] * row["wavelength_m"] - step)
                      <= slip_threshold_cycles * row["wavelength_m"] for row in group]
            common = len({row["satellite_id"] for row in group}) >= min_clock_witnesses and sum(agrees) / len(group) >= 0.75 and (
                abs(step) > slip_threshold_cycles * statistics.median(row["wavelength_m"] for row in group))
            for row in group:
                if common:
                    row["clock_step_candidate"] = True
                    row["common_clock_step_m"] = step
                    row["phase_doppler_adjusted_cycles"] -= step / row["wavelength_m"]
                    row["reason_codes"].append("receiver_clock_discontinuity_candidate")
                if abs(row["phase_doppler_adjusted_cycles"]) > slip_threshold_cycles:
                    row["slip_suspect"] = True
                    row["reason_codes"].append("phase_doppler_discontinuity")
        records.extend(current)
    if not epoch_count or not records:
        raise ValueError("no corrected observation rows to analyze")
    counts = Counter(reason for row in records for reason in row["reason_codes"])
    satellites = {}
    for satellite in sorted({row["satellite_id"] for row in records}):
        rows = [row for row in records if row["satellite_id"] == satellite]
        residuals = [abs(row["clock_removed_code_residual_m"]) for row in rows
                     if row["clock_removed_code_residual_m"] is not None]
        snrs = [row["snr_dbhz"] for row in rows if row["snr_dbhz"] is not None]
        satellites[satellite] = {"rows": len(rows), "signals": sorted({row["signal_id"] for row in rows}),
                                "slip_suspect_rows": sum(row["slip_suspect"] for row in rows),
                                "code_abs_residual_p95_m": quantile(residuals, 0.95),
                                "snr_median_dbhz": statistics.median(snrs) if snrs else None}
    summary = {"schema_version": 1, "epochs": epoch_count, "valid_solution_epochs": valid_count,
               "observation_rows": len(records), "satellites": satellites,
               "slip_suspect_rows": sum(row["slip_suspect"] for row in records),
               "phase_doppler_assessed_rows": sum(row["continuity_assessed"] for row in records),
               "clock_step_candidate_rows": sum(row["clock_step_candidate"] for row in records),
               "reason_counts": dict(counts),
               "settings": {"slip_threshold_cycles": slip_threshold_cycles, "max_gap_s": max_gap_s,
                            "min_clock_witnesses": min_clock_witnesses},
               "residual_contract": "code residuals centered by native receiver clock group at the SPP position; not solver postfit residuals",
               "slip_contract": "source LLI/lock flags and phase plus trapezoidal RINEX Doppler; diagnostic candidates, not confirmed integer slips",
               "reference_used": False}
    return records, summary
