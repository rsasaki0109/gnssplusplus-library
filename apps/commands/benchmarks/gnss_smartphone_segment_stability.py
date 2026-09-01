#!/usr/bin/env python3
"""Truth-free segment stability gate for smartphone trajectory smoothing.

The existing Kalman/RTS result is retained for stable static segments.  An
unstable segment is replaced wholesale by the canonical raw/Hatch POS.  A
device epoch with no corresponding raw row is explicitly interpolated from
the nearest bracketing raw rows (or held at the nearest edge row when no
bracket exists).  No truth, labels, or route-specific accuracy value enters
the decision.
"""

from __future__ import annotations

from bisect import bisect_left
import math
from typing import Any

import numpy as np

import gnss_smartphone_trajectory_smoother as smoother


SCHEMA_VERSION = "smartphone-segment-stability-gate.v1"


class SegmentStabilityError(ValueError):
    """Raised for malformed or numerically unsafe segment fallback inputs."""


def _validate_thresholds(
    max_consecutive_rejects: int,
    max_prediction_duration_s: float | None,
    reject_fraction_max: float | None,
) -> None:
    if (
        isinstance(max_consecutive_rejects, bool)
        or not isinstance(max_consecutive_rejects, int)
        or max_consecutive_rejects <= 0
    ):
        raise SegmentStabilityError("max_consecutive_rejects must be a positive integer")
    if max_prediction_duration_s is not None and (
        not math.isfinite(max_prediction_duration_s)
        or max_prediction_duration_s <= 0.0
    ):
        raise SegmentStabilityError(
            "max_prediction_duration_s must be finite and positive or None"
        )
    if reject_fraction_max is not None and (
        not math.isfinite(reject_fraction_max)
        or not 0.0 <= reject_fraction_max <= 1.0
    ):
        raise SegmentStabilityError(
            "reject_fraction_max must be within [0, 1] or None"
        )


def _segment_runs(rows: list[smoother.SmoothedRow]) -> list[tuple[int, int, int]]:
    if not rows:
        raise SegmentStabilityError("baseline smoother produced no rows")
    runs: list[tuple[int, int, int]] = []
    start = 0
    current_segment = rows[0].segment_id
    for index in range(1, len(rows)):
        if rows[index].segment_id != current_segment:
            runs.append((start, index, current_segment))
            start = index
            current_segment = rows[index].segment_id
    runs.append((start, len(rows), current_segment))
    return runs


def _segment_metrics(
    rows: list[smoother.SmoothedRow],
    epochs: list[int],
    start: int,
    end: int,
) -> dict[str, Any]:
    rejected = sum(1 for row in rows[start:end] if row.outlier_rejected)
    maximum_reject_run = 0
    current_reject_run = 0
    maximum_prediction_duration = 0.0
    prediction_duration = 0.0
    max_innovation = 0.0
    saw_innovation = False
    for index in range(start, end):
        row = rows[index]
        if row.outlier_rejected:
            current_reject_run += 1
        else:
            maximum_reject_run = max(maximum_reject_run, current_reject_run)
            current_reject_run = 0
        if row.innovation_sigma is not None and math.isfinite(row.innovation_sigma):
            max_innovation = max(max_innovation, float(row.innovation_sigma))
            saw_innovation = True
        # Both a gated observation and a device-key interpolation are
        # prediction-only rows in the baseline result.  The duration metric
        # deliberately measures this whole truth-free prediction interval.
        if not row.measurement_used:
            if index > start:
                dt = (epochs[index] - epochs[index - 1]) / 1000.0
                if not math.isfinite(dt) or dt <= 0.0:
                    raise SegmentStabilityError("device epoch keys are not increasing")
                prediction_duration += dt
            maximum_prediction_duration = max(
                maximum_prediction_duration, prediction_duration
            )
        else:
            prediction_duration = 0.0
    maximum_reject_run = max(maximum_reject_run, current_reject_run)
    length = end - start
    return {
        "start_index": start,
        "end_index": end - 1,
        "segment_id": rows[start].segment_id,
        "epoch_count": length,
        "rejected_epochs": rejected,
        "reject_fraction": rejected / length if length else 0.0,
        "max_consecutive_rejects": maximum_reject_run,
        "max_normalized_innovation_sigma": max_innovation if saw_innovation else None,
        "max_prediction_duration_s": maximum_prediction_duration,
    }


def _raw_row(
    raw: smoother.PositionRow,
    baseline: smoother.SmoothedRow,
    measurement_floor_m: float,
) -> smoother.SmoothedRow:
    return smoother.SmoothedRow(
        timestamp_ms=raw.timestamp_ms,
        week=raw.week,
        tow=raw.tow,
        ecef=raw.ecef.astype(float, copy=True),
        latitude=raw.latitude,
        longitude=raw.longitude,
        height=raw.height,
        status=raw.status,
        satellites=raw.satellites,
        pdop=raw.pdop,
        ratio=raw.ratio,
        fixed_ambiguities=raw.fixed_ambiguities,
        iterations=raw.iterations,
        source="raw_fallback",
        segment_id=baseline.segment_id,
        measurement_used=True,
        outlier_rejected=False,
        innovation_sigma=None,
        position_sigma_m=smoother._measurement_sigma(raw, measurement_floor_m),
    )


def _interpolated_row(
    timestamp_ms: int,
    baseline: smoother.SmoothedRow,
    previous: smoother.PositionRow,
    following: smoother.PositionRow | None,
    measurement_floor_m: float,
    leap_seconds: int,
) -> smoother.SmoothedRow:
    if following is None or following.timestamp_ms == previous.timestamp_ms:
        ecef = previous.ecef.astype(float, copy=True)
        source = "raw_nearest"
    else:
        alpha = (timestamp_ms - previous.timestamp_ms) / (
            following.timestamp_ms - previous.timestamp_ms
        )
        if not math.isfinite(alpha):
            raise SegmentStabilityError("raw interpolation fraction is non-finite")
        alpha = min(1.0, max(0.0, alpha))
        ecef = previous.ecef + alpha * (following.ecef - previous.ecef)
        source = "raw_interpolated"
    if not np.isfinite(ecef).all():
        raise SegmentStabilityError("raw fallback ECEF is non-finite")
    latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(ecef)
    week, tow = smoother._device_time_to_week_tow(timestamp_ms, leap_seconds)
    return smoother.SmoothedRow(
        timestamp_ms=timestamp_ms,
        week=week,
        tow=tow,
        ecef=ecef.astype(float, copy=True),
        latitude=math.degrees(latitude),
        longitude=math.degrees(longitude),
        height=height,
        status=smoother.PROPAGATED_STATUS,
        satellites=0,
        pdop=0.0,
        ratio=0.0,
        fixed_ambiguities=0,
        iterations=0,
        source=source,
        segment_id=baseline.segment_id,
        measurement_used=False,
        outlier_rejected=False,
        innovation_sigma=None,
        position_sigma_m=max(measurement_floor_m, 1.0),
    )


def _fallback_row(
    timestamp_ms: int,
    baseline: smoother.SmoothedRow,
    raw_rows: list[smoother.PositionRow],
    raw_timestamps: list[int],
    raw_by_timestamp: dict[int, smoother.PositionRow],
    measurement_floor_m: float,
    leap_seconds: int,
) -> smoother.SmoothedRow:
    exact = raw_by_timestamp.get(timestamp_ms)
    if exact is not None:
        return _raw_row(exact, baseline, measurement_floor_m)
    insertion = bisect_left(raw_timestamps, timestamp_ms)
    if insertion <= 0:
        previous = raw_rows[0]
        following = None
    elif insertion >= len(raw_rows):
        previous = raw_rows[-1]
        following = None
    else:
        previous = raw_rows[insertion - 1]
        following = raw_rows[insertion]
    return _interpolated_row(
        timestamp_ms,
        baseline,
        previous,
        following,
        measurement_floor_m,
        leap_seconds,
    )


def apply_segment_stability(
    baseline_result: smoother.SmoothingResult,
    positions: list[smoother.PositionRow],
    device_epochs: list[int],
    *,
    max_consecutive_rejects: int,
    max_prediction_duration_s: float | None = None,
    reject_fraction_max: float | None = None,
    measurement_floor_m: float = 1.0,
    leap_seconds: int = smoother.DEFAULT_GPS_UTC_LEAP_SECONDS,
) -> tuple[smoother.SmoothingResult, dict[str, Any]]:
    """Apply a static-segment stability decision without using truth."""

    _validate_thresholds(
        max_consecutive_rejects,
        max_prediction_duration_s,
        reject_fraction_max,
    )
    if len(baseline_result.rows) != len(device_epochs):
        raise SegmentStabilityError("baseline row count differs from device epochs")
    if not positions:
        raise SegmentStabilityError("raw position list is empty")
    if any(b.timestamp_ms <= a.timestamp_ms for a, b in zip(positions, positions[1:])):
        raise SegmentStabilityError("raw position timestamps are not strictly increasing")
    raw_by_timestamp: dict[int, smoother.PositionRow] = {}
    for row in positions:
        if row.timestamp_ms in raw_by_timestamp:
            raise SegmentStabilityError("duplicate raw position timestamp")
        raw_by_timestamp[row.timestamp_ms] = row
    raw_timestamps = [row.timestamp_ms for row in positions]
    final_rows = list(baseline_result.rows)
    decisions: list[dict[str, Any]] = []
    fallback_epochs = 0
    raw_fallback_epochs = 0
    raw_interpolated_epochs = 0
    raw_nearest_epochs = 0
    for start, end, segment_id in _segment_runs(baseline_result.rows):
        metrics = _segment_metrics(baseline_result.rows, device_epochs, start, end)
        reasons: list[str] = []
        if metrics["max_consecutive_rejects"] > max_consecutive_rejects:
            reasons.append("max_consecutive_rejects_exceeded")
        if (
            max_prediction_duration_s is not None
            and metrics["max_prediction_duration_s"] > max_prediction_duration_s
        ):
            reasons.append("max_prediction_duration_exceeded")
        if (
            reject_fraction_max is not None
            and metrics["reject_fraction"] > reject_fraction_max
        ):
            reasons.append("reject_fraction_exceeded")
        stable = not reasons
        segment_decision = {
            **metrics,
            "decision": "stable_rts" if stable else "unstable_raw_fallback",
            "stable": stable,
            "reason": "within_fixed_thresholds" if stable else "+".join(reasons),
            "fallback_epochs": 0,
            "raw_fallback_epochs": 0,
            "raw_interpolated_epochs": 0,
            "raw_nearest_epochs": 0,
        }
        if not stable:
            for index in range(start, end):
                replacement = _fallback_row(
                    device_epochs[index],
                    baseline_result.rows[index],
                    positions,
                    raw_timestamps,
                    raw_by_timestamp,
                    measurement_floor_m,
                    leap_seconds,
                )
                final_rows[index] = replacement
                fallback_epochs += 1
                segment_decision["fallback_epochs"] += 1
                if replacement.source == "raw_fallback":
                    raw_fallback_epochs += 1
                    segment_decision["raw_fallback_epochs"] += 1
                elif replacement.source == "raw_interpolated":
                    raw_interpolated_epochs += 1
                    segment_decision["raw_interpolated_epochs"] += 1
                else:
                    raw_nearest_epochs += 1
                    segment_decision["raw_nearest_epochs"] += 1
        decisions.append(segment_decision)

    result = smoother.SmoothingResult(
        rows=final_rows,
        origin_ecef=baseline_result.origin_ecef,
        origin_latitude=baseline_result.origin_latitude,
        origin_longitude=baseline_result.origin_longitude,
        origin_height=baseline_result.origin_height,
        selected_device_epochs=baseline_result.selected_device_epochs,
        measured_epochs=sum(1 for row in final_rows if row.measurement_used),
        synthesized_epochs=sum(1 for row in final_rows if not row.measurement_used),
        outlier_rejections=sum(1 for row in final_rows if row.outlier_rejected),
        segment_count=baseline_result.segment_count,
        max_input_gap_s=baseline_result.max_input_gap_s,
        max_position_gap_s=baseline_result.max_position_gap_s,
        reset_indices=baseline_result.reset_indices,
        numerical_fallbacks=baseline_result.numerical_fallbacks,
        elapsed_s=baseline_result.elapsed_s,
        reacquisition_indices=(),
        max_consecutive_rejects=baseline_result.max_consecutive_rejects,
        max_prediction_duration_s=baseline_result.max_prediction_duration_s,
    )
    stable_count = sum(1 for decision in decisions if decision["stable"])
    stability = {
        "schema_version": SCHEMA_VERSION,
        "decision_policy": {
            "stable": "RTS rows retained when every fixed threshold is within bound",
            "unstable": "all segment coordinates replaced by canonical raw/Hatch POS",
            "missing_device_key": "explicit raw interpolation from nearest bracketing rows; edge uses nearest raw row",
            "truth_used": False,
        },
        "thresholds": {
            "max_consecutive_rejects": max_consecutive_rejects,
            "max_prediction_duration_s": max_prediction_duration_s,
            "reject_fraction_max": reject_fraction_max,
        },
        "population": {
            "segment_count": len(decisions),
            "stable_segment_count": stable_count,
            "unstable_segment_count": len(decisions) - stable_count,
            "fallback_epochs": fallback_epochs,
            "raw_fallback_epochs": raw_fallback_epochs,
            "raw_interpolated_epochs": raw_interpolated_epochs,
            "raw_nearest_epochs": raw_nearest_epochs,
        },
        "segments": decisions,
    }
    return result, stability
