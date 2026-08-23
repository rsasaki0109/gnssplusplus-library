"""Truth-qualified metrics for static-position application workflows."""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Any


def parse_ecef(text: str) -> tuple[float, float, float]:
    try:
        values = tuple(float(item.strip()) for item in text.split(","))
    except ValueError as exc:
        raise ValueError("truth ECEF must be X,Y,Z in metres") from exc
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("truth ECEF must be three finite comma-separated values")
    return values  # type: ignore[return-value]


def _ecef_lat_lon(ecef: tuple[float, float, float]) -> tuple[float, float]:
    x, y, z = ecef
    a = 6378137.0
    f = 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p * (1.0 - e2))
    for _ in range(12):
        sin_lat = math.sin(lat)
        n = a / math.sqrt(1.0 - e2 * sin_lat * sin_lat)
        updated = math.atan2(z + e2 * n * sin_lat, p)
        if abs(updated - lat) < 1e-14:
            lat = updated
            break
        lat = updated
    return lat, lon


def _rotation_ecef_to_enu(ecef: tuple[float, float, float]) -> list[list[float]]:
    lat, lon = _ecef_lat_lon(ecef)
    slat, clat = math.sin(lat), math.cos(lat)
    slon, clon = math.sin(lon), math.cos(lon)
    return [
        [-slon, clon, 0.0],
        [-slat * clon, -slat * slon, clat],
        [clat * clon, clat * slon, slat],
    ]


def _mat_vec(matrix: list[list[float]], vector: tuple[float, float, float]) -> list[float]:
    return [sum(row[index] * vector[index] for index in range(3)) for row in matrix]


def _mat_cov(matrix: list[list[float]], covariance: list[list[float]]) -> list[list[float]]:
    interim = [
        [sum(matrix[i][k] * covariance[k][j] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]
    return [
        [sum(interim[i][k] * matrix[j][k] for k in range(3)) for j in range(3)]
        for i in range(3)
    ]


def static_truth_metrics(
    records: Iterable[Mapping[str, Any]],
    truth_ecef_m: tuple[float, float, float],
    *,
    accepted_statuses: set[int] | None = None,
) -> dict[str, Any]:
    all_rows = list(records)
    selected = [
        row for row in all_rows
        if accepted_statuses is None or int(row["status"]) in accepted_statuses
    ]
    if not selected:
        raise ValueError("no solution epochs in the selected static population")
    xyz = [tuple(float(row[name]) for name in ("x", "y", "z")) for row in selected]
    mean = tuple(sum(row[index] for row in xyz) / len(xyz) for index in range(3))
    divisor = max(1, len(xyz) - 1)
    covariance = [
        [
            sum((row[i] - mean[i]) * (row[j] - mean[j]) for row in xyz) / divisor
            for j in range(3)
        ]
        for i in range(3)
    ]
    rotation = _rotation_ecef_to_enu(truth_ecef_m)
    delta = tuple(mean[index] - truth_ecef_m[index] for index in range(3))
    enu = _mat_vec(rotation, delta)
    covariance_enu = _mat_cov(rotation, covariance)
    errors = [math.dist(row, truth_ecef_m) for row in xyz]
    return {
        "population": {
            "all_solution_epochs": len(all_rows),
            "accepted_epochs": len(selected),
            "accepted_statuses": sorted(accepted_statuses) if accepted_statuses is not None else None,
        },
        "truth_ecef_m": list(truth_ecef_m),
        "estimated_ecef_mean_m": list(mean),
        "enu_delta_m": {"east": enu[0], "north": enu[1], "up": enu[2]},
        "horizontal_error_m": math.hypot(enu[0], enu[1]),
        "vertical_error_m": abs(enu[2]),
        "error_3d_m": math.sqrt(sum(value * value for value in enu)),
        "epoch_error_3d_mean_m": sum(errors) / len(errors),
        "epoch_error_3d_max_m": max(errors),
        "empirical_covariance_ecef_m2": covariance,
        "empirical_covariance_enu_m2": covariance_enu,
        "covariance_role": "empirical repeatability of accepted epoch coordinates; not solver posterior covariance",
    }
