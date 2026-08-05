#!/usr/bin/env python3
"""Score the Doppler-only dead-reckoning witness for FGO LAMBDA candidates."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


WGS84_A = 6378137.0
WGS84_E2 = 6.69437999014e-3
CORRECT_ERROR_M = 0.5
MINIMUM_EVALUATED_FLOAT_CANDIDATES = 100
MINIMUM_ACCEPTED_CORRECT_FLOAT_CANDIDATES = 60
MAXIMUM_ACCEPTED_WRONG_FLOAT_CANDIDATES = 0


def as_float(row: dict[str, str], name: str) -> float:
    value = row.get(name, "")
    return float(value) if value else 0.0


def as_int(row: dict[str, str], name: str) -> int:
    return int(as_float(row, name))


def ecef_lat_lon(x_m: float, y_m: float, z_m: float) -> tuple[float, float]:
    longitude = math.atan2(y_m, x_m)
    p = math.hypot(x_m, y_m)
    latitude = math.atan2(z_m, p * (1.0 - WGS84_E2))
    for _ in range(8):
        sin_latitude = math.sin(latitude)
        normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_latitude**2)
        height = p / max(math.cos(latitude), 1e-12) - normal
        latitude = math.atan2(
            z_m, p * (1.0 - WGS84_E2 * normal / (normal + height))
        )
    return latitude, longitude


def ecef_delta_to_enu(
    delta: tuple[float, float, float], latitude: float, longitude: float
) -> tuple[float, float, float]:
    dx, dy, dz = delta
    sin_lat, cos_lat = math.sin(latitude), math.cos(latitude)
    sin_lon, cos_lon = math.sin(longitude), math.cos(longitude)
    return (
        -sin_lon * dx + cos_lon * dy,
        -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz,
        cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz,
    )


def candidate_error(row: dict[str, str]) -> tuple[float, float]:
    solution = tuple(as_float(row, name) for name in ("x_ecef_m", "y_ecef_m", "z_ecef_m"))
    candidate = tuple(
        as_float(row, name)
        for name in (
            "lambda_candidate_x_ecef_m",
            "lambda_candidate_y_ecef_m",
            "lambda_candidate_z_ecef_m",
        )
    )
    latitude, longitude = ecef_lat_lon(*solution)
    delta_enu = ecef_delta_to_enu(
        tuple(candidate[i] - solution[i] for i in range(3)), latitude, longitude
    )
    candidate_enu = tuple(
        as_float(row, name) + delta_enu[i]
        for i, name in enumerate(("e_pos_m", "n_pos_m", "u_pos_m"))
    )
    reference_enu = tuple(
        as_float(row, name) for name in ("ref_e_pos_m", "ref_n_pos_m", "ref_u_pos_m")
    )
    error = tuple(candidate_enu[i] - reference_enu[i] for i in range(3))
    return math.hypot(error[0], error[1]), math.sqrt(sum(value**2 for value in error))


def analyze_rows(rows: list[dict[str, str]]) -> dict[str, Any]:
    population = {
        "candidate_epochs": 0,
        "evaluated_candidates": 0,
        "evaluated_float_candidates": 0,
        "evaluated_fixed_candidates": 0,
        "float_candidate_correct": 0,
        "float_candidate_wrong": 0,
        "accepted_correct_float_candidates": 0,
        "accepted_wrong_float_candidates": 0,
        "rejected_correct_float_candidates": 0,
        "rejected_wrong_float_candidates": 0,
    }
    accepted_correct_tows: list[float] = []
    accepted_wrong_tows: list[float] = []
    for row in rows:
        if as_int(row, "lambda_candidate_valid") == 0:
            continue
        population["candidate_epochs"] += 1
        if as_int(row, "external_dr_eval") == 0:
            continue
        population["evaluated_candidates"] += 1
        is_float = row.get("status", "").upper() != "FIXED"
        population[
            "evaluated_float_candidates" if is_float else "evaluated_fixed_candidates"
        ] += 1
        if not is_float:
            continue
        _, error_3d_m = candidate_error(row)
        correct = error_3d_m <= CORRECT_ERROR_M
        accepted = as_int(row, "external_dr_accept") > 0
        population["float_candidate_correct" if correct else "float_candidate_wrong"] += 1
        population[
            ("accepted_" if accepted else "rejected_")
            + ("correct_" if correct else "wrong_")
            + "float_candidates"
        ] += 1
        if accepted:
            (accepted_correct_tows if correct else accepted_wrong_tows).append(
                as_float(row, "tow")
            )

    gate_passed = (
        population["evaluated_float_candidates"]
        >= MINIMUM_EVALUATED_FLOAT_CANDIDATES
        and population["accepted_correct_float_candidates"]
        >= MINIMUM_ACCEPTED_CORRECT_FLOAT_CANDIDATES
        and population["accepted_wrong_float_candidates"]
        <= MAXIMUM_ACCEPTED_WRONG_FLOAT_CANDIDATES
    )
    return {
        "rows": len(rows),
        "requirements": {
            "correct_candidate_3d_error_max_m": CORRECT_ERROR_M,
            "minimum_evaluated_float_candidates": MINIMUM_EVALUATED_FLOAT_CANDIDATES,
            "minimum_accepted_correct_float_candidates": (
                MINIMUM_ACCEPTED_CORRECT_FLOAT_CANDIDATES
            ),
            "maximum_accepted_wrong_float_candidates": (
                MAXIMUM_ACCEPTED_WRONG_FLOAT_CANDIDATES
            ),
        },
        "population": population,
        "accepted_correct_float_tows": accepted_correct_tows,
        "accepted_wrong_float_tows": accepted_wrong_tows,
        "gate_passed": gate_passed,
    }


def render_markdown(summary: dict[str, Any]) -> str:
    p = summary["population"]
    return "\n".join(
        [
            "# FGO external Doppler-DR witness audit",
            "",
            f"Verdict: **{'PASS' if summary['gate_passed'] else 'FAIL'}**",
            "",
            "| Population | Count |",
            "|---|---:|",
            f"| Candidate epochs | {p['candidate_epochs']} |",
            f"| Evaluated FLOAT candidates | {p['evaluated_float_candidates']} |",
            f"| Correct / wrong FLOAT candidates | {p['float_candidate_correct']} / {p['float_candidate_wrong']} |",
            f"| Accepted correct FLOAT candidates | {p['accepted_correct_float_candidates']} |",
            f"| Accepted wrong FLOAT candidates | {p['accepted_wrong_float_candidates']} |",
            "",
            "The frozen promotion gate requires at least 100 evaluated FLOAT candidates, "
            "at least 60 accepted correct candidates, and zero accepted wrong candidates.",
            "",
        ]
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--epoch-csv", type=Path, required=True)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--markdown", type=Path)
    args = parser.parse_args()
    with args.epoch_csv.open(newline="", encoding="utf-8-sig") as stream:
        summary = analyze_rows(list(csv.DictReader(stream)))
    rendered = json.dumps(summary, indent=2, sort_keys=True, allow_nan=False)
    print(rendered)
    if args.json:
        args.json.write_text(rendered + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.write_text(render_markdown(summary), encoding="utf-8")
    return 0 if summary["gate_passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
