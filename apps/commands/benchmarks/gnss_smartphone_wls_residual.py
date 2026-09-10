#!/usr/bin/env python3
"""Truth-free, experimental robust post-processing for handset WLS POS.

The candidate set is deliberately tiny and fixed before validation data is
opened: component-wise centered medians over 3 or 5 epochs, with an optional
small along-track shift.  Heading is derived only from the WLS trajectory and
the output is published together with an integrity manifest.  This module
never accepts ground truth or a device model.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

from support.gnss_runtime import application_root


ROOT = application_root(__file__)
BENCHMARK_DIR = ROOT / "apps" / "commands" / "benchmarks"
if str(BENCHMARK_DIR) not in sys.path:
    sys.path.insert(0, str(BENCHMARK_DIR))
import gnss_smartphone_trajectory_smoother as smoother  # noqa: E402


SCHEMA_VERSION = "smartphone-wls-residual-candidate.v1"
MANIFEST_SCHEMA_VERSION = "smartphone-wls-residual-candidate-manifest.v1"
LEAP_SECONDS = 18
SHIFT_VALUES_M = (-1.0, 0.0, 1.0)


def _shift_token(shift_m: float) -> str:
    if shift_m < 0.0:
        return "m1"
    if shift_m > 0.0:
        return "p1"
    return "0"


@dataclass(frozen=True)
class CandidateSpec:
    candidate_id: str
    robust_window: int
    along_shift_m: float


PREDECLARED_CANDIDATES = tuple(
    CandidateSpec(f"raw_shift_{_shift_token(shift)}", 1, shift)
    for shift in SHIFT_VALUES_M
) + tuple(
    CandidateSpec(
        f"median{window}_shift_{_shift_token(shift)}",
        window,
        shift,
    )
    for window in (3, 5)
    for shift in SHIFT_VALUES_M
)
CANDIDATES_BY_ID = {candidate.candidate_id: candidate for candidate in PREDECLARED_CANDIDATES}


class WlsResidualCandidateError(ValueError):
    """Raised when a truth-free candidate contract cannot be proven."""


def _sha256(path: Path) -> str:
    if not path.is_file():
        raise WlsResidualCandidateError(f"missing artifact: {path}")
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    smoother._atomic_write(path, content)


def candidate_spec(candidate_id: str) -> CandidateSpec:
    try:
        return CANDIDATES_BY_ID[candidate_id]
    except KeyError as exc:
        raise WlsResidualCandidateError(
            f"candidate is not in the predeclared set: {candidate_id}"
        ) from exc


def _enu_basis(ecef: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    latitude, longitude, _ = smoother._wgs84_ecef_to_geodetic(ecef)
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    east = np.array((-sin_lon, cos_lon, 0.0), dtype=float)
    north = np.array(
        (-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat), dtype=float
    )
    up = np.array((cos_lat * cos_lon, cos_lat * sin_lon, sin_lat), dtype=float)
    return east, north, up


def _centered_componentwise_median(ecef: np.ndarray, window: int) -> np.ndarray:
    if window == 1:
        return ecef.copy()
    radius = window // 2
    output = np.empty_like(ecef)
    for index in range(len(ecef)):
        output[index] = np.median(
            ecef[max(0, index - radius) : min(len(ecef), index + radius + 1)], axis=0
        )
    return output


def transform_ecef(
    rows: list[smoother.PositionRow], spec: CandidateSpec
) -> tuple[np.ndarray, dict[str, Any]]:
    """Apply a fixed candidate using only the input POS trajectory."""

    if not rows:
        raise WlsResidualCandidateError("WLS position input is empty")
    source = np.asarray([row.ecef for row in rows], dtype=float)
    if source.ndim != 2 or source.shape[1] != 3 or not np.isfinite(source).all():
        raise WlsResidualCandidateError("WLS position input contains non-finite ECEF")
    filtered = _centered_componentwise_median(source, spec.robust_window)
    output = filtered.copy()
    heading_fallbacks = 0
    for index, base in enumerate(filtered):
        if spec.along_shift_m == 0.0:
            continue
        east, north, up = _enu_basis(base)
        if index == 0:
            delta = filtered[1] - filtered[0] if len(filtered) > 1 else np.zeros(3)
        elif index == len(filtered) - 1:
            delta = filtered[-1] - filtered[-2]
        else:
            delta = filtered[index + 1] - filtered[index - 1]
        tangent = delta - up * float(np.dot(delta, up))
        tangent_enu = np.array((np.dot(tangent, east), np.dot(tangent, north)))
        norm = float(np.linalg.norm(tangent_enu))
        if not math.isfinite(norm) or norm <= 1.0e-9:
            heading_fallbacks += 1
            continue
        unit = (east * tangent_enu[0] + north * tangent_enu[1]) / norm
        output[index] = base + spec.along_shift_m * unit
    if not np.isfinite(output).all():
        raise WlsResidualCandidateError("candidate output contains non-finite ECEF")
    return output, {
        "heading_fallback_count": heading_fallbacks,
        "input_rows": len(rows),
        "output_rows": len(output),
        "all_values_finite": True,
    }


def _position_lines(
    rows: list[smoother.PositionRow], ecef: np.ndarray, candidate_id: str
) -> bytes:
    lines = [
        "% LibGNSS++ truth-free experimental handset WLS residual candidate",
        "% Columns: GPS_Week GPS_TOW X(m) Y(m) Z(m) Lat(deg) Lon(deg) Height(m) Status Satellites PDOP Ratio FixedAmbiguities Iterations",
        f"% Candidate: {candidate_id}; truth and device model are not used.",
    ]
    for row, coordinate in zip(rows, ecef):
        latitude, longitude, height = smoother._wgs84_ecef_to_geodetic(coordinate)
        if not all(math.isfinite(value) for value in (latitude, longitude, height)):
            raise WlsResidualCandidateError("candidate geodetic output is non-finite")
        lines.append(
            f"{row.week:d} {row.tow:.6f} "
            f"{coordinate[0]:.6f} {coordinate[1]:.6f} {coordinate[2]:.6f} "
            f"{math.degrees(latitude):.9f} {math.degrees(longitude):.9f} {height:.6f} "
            f"{row.status:d} {row.satellites:d} {row.pdop:.6f} "
            f"{row.ratio:.6f} {row.fixed_ambiguities:d} {row.iterations:d}"
        )
    return ("\n".join(lines) + "\n").encode("ascii")


def write_candidate(
    input_path: Path,
    output_path: Path,
    candidate_id: str,
    *,
    manifest_path: Path | None = None,
    leap_seconds: int = LEAP_SECONDS,
) -> dict[str, Any]:
    """Atomically publish a truth-free transformed POS and manifest."""

    spec = candidate_spec(candidate_id)
    rows = smoother._read_positions(input_path, leap_seconds)
    output, transform_summary = transform_ecef(rows, spec)
    _atomic_write(output_path, _position_lines(rows, output, candidate_id))
    if manifest_path is None:
        manifest_path = output_path.with_name(output_path.name + ".manifest.json")
    payload = {
        "schema_version": MANIFEST_SCHEMA_VERSION,
        "truth_free": True,
        "truth_used": False,
        "device_model_used": False,
        "candidate": {
            "candidate_id": spec.candidate_id,
            "robust_estimator": "component-wise centered median",
            "robust_window_epochs": spec.robust_window,
            "along_track_shift_m": spec.along_shift_m,
            "heading_source": "input WLS ECEF trajectory",
            "heading_frame": "local WGS84 tangent plane",
        },
        "inputs": {
            "wls_position": {
                "path": str(input_path),
                "sha256": _sha256(input_path),
            }
        },
        "transform": transform_summary,
        "output": {
            "path": str(output_path),
            "sha256": _sha256(output_path),
            "bytes": output_path.stat().st_size,
        },
    }
    _atomic_write(
        manifest_path,
        (json.dumps(payload, indent=2, sort_keys=True) + "\n").encode("utf-8"),
    )
    payload["manifest"] = {
        "path": str(manifest_path),
        "sha256": _sha256(manifest_path),
        "bytes": manifest_path.stat().st_size,
    }
    return payload


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(prog=os.environ.get("GNSS_CLI_NAME", "gnss smartphone-wls-residual"))
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--candidate-id", choices=tuple(CANDIDATES_BY_ID), required=True)
    parser.add_argument("--leap-seconds", type=int, default=LEAP_SECONDS)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        write_candidate(
            args.input,
            args.output,
            args.candidate_id,
            manifest_path=args.manifest,
            leap_seconds=args.leap_seconds,
        )
    except (OSError, ValueError, smoother.SmootherError, WlsResidualCandidateError) as exc:
        print(f"smartphone WLS residual candidate failed: {exc}", file=sys.stderr)
        return 1
    print(f"Smartphone WLS residual candidate complete: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
