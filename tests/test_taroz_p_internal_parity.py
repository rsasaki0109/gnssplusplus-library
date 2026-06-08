#!/usr/bin/env python3
"""Optional parity checks against taroz MATLAB P dogfood dumps."""

from __future__ import annotations

import csv
from collections import Counter
import json
import math
import os
from pathlib import Path
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]


def configured_path(env_name: str, default_relative_path: str) -> Path:
    configured = os.environ.get(env_name)
    if configured:
        path = Path(configured)
        return path if path.is_absolute() else ROOT_DIR / path
    return ROOT_DIR / default_relative_path


CPP_DIR = configured_path(
    "GNSSPP_TAROZ_P_CPP_DIR",
    "output/dogfood/taroz_p_dogfood_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_P_MATLAB_DIR",
    "output/dogfood/taroz_matlab_p_debug",
)


def finite_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def read_csv_by_tow(path: Path) -> dict[int, dict[str, str]]:
    with path.open(newline="") as handle:
        return {
            round(float(row["gps_tow"])): row
            for row in csv.DictReader(handle)
        }


def read_pos_by_tow(path: Path) -> dict[int, tuple[float, float, float, int]]:
    rows: dict[int, tuple[float, float, float, int]] = {}
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            rows[round(float(parts[1]))] = (
                float(parts[2]),
                float(parts[3]),
                float(parts[4]),
                int(float(parts[8])),
            )
    return rows


def read_pos_rows(path: Path) -> list[dict[str, str]]:
    fields = (
        "gps_week",
        "gps_tow",
        "x_m",
        "y_m",
        "z_m",
        "lat_deg",
        "lon_deg",
        "height_m",
        "status",
        "satellites",
        "pdop",
        "ratio",
        "fixed_ambiguities",
        "iterations",
    )
    rows: list[dict[str, str]] = []
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) != len(fields):
                raise ValueError(f"unexpected taroz P pos row width in {path}: {line.rstrip()}")
            rows.append(dict(zip(fields, parts)))
    return rows


def count_valid_pseudorange_rows(path: Path, allowed_tows: set[int]) -> int:
    count = 0
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            tow = round(float(row["gps_tow"]))
            if tow in allowed_tows and row.get("valid_pseudorange", "").lower() in {"1", "true"}:
                count += 1
    return count


def read_raw_factor_debug_by_key(path: Path) -> dict[tuple[int, str], dict[str, str]]:
    rows: dict[tuple[int, str], dict[str, str]] = {}
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            key = (round(float(row["gps_tow"])), row["satellite"])
            rows[key] = row
    return rows


def read_valid_taroz_p_sat_rows_by_key(
    path: Path,
    allowed_tows: set[int],
) -> dict[tuple[int, str], dict[str, str]]:
    rows: dict[tuple[int, str], dict[str, str]] = {}
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            tow = round(float(row["gps_tow"]))
            if tow not in allowed_tows:
                continue
            if row.get("valid_pseudorange", "").lower() not in {"1", "true"}:
                continue
            key = (tow, row["satellite"])
            rows[key] = row
    return rows


def read_json(path: Path) -> dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def max_abs_diff(
    keys: list[tuple[int, str]],
    left: dict[tuple[int, str], dict[str, str]],
    right: dict[tuple[int, str], dict[str, str]],
    left_column: str,
    right_column: str,
) -> float:
    return max(
        abs(float(left[key][left_column]) - float(right[key][right_column]))
        for key in keys
    )


def mean_abs_diff(
    keys: list[tuple[int, str]],
    left: dict[tuple[int, str], dict[str, str]],
    right: dict[tuple[int, str], dict[str, str]],
    left_column: str,
    right_column: str,
) -> float:
    return sum(
        abs(float(left[key][left_column]) - float(right[key][right_column]))
        for key in keys
    ) / len(keys)


class TarozPInternalParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [str(path.relative_to(ROOT_DIR)) for path in paths if not path.exists()]
        if missing:
            self.skipTest("missing optional taroz P dogfood files: " + ", ".join(missing))

    def test_final_positions_stay_close_to_taroz_p_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "fgo_taroz_p_summary.json"
        cpp_pos_path = CPP_DIR / "fgo_taroz_p.pos"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_est.csv"
        self.require_dogfood_files(cpp_summary_path, cpp_pos_path, taroz_epoch_path)
        if read_json(cpp_summary_path).get("debug_problem_only") is True:
            self.skipTest("debug-only taroz P output intentionally keeps seed positions")

        cpp_positions = read_pos_by_tow(cpp_pos_path)
        taroz_epochs = read_csv_by_tow(taroz_epoch_path)
        common_tows = sorted(set(cpp_positions) & set(taroz_epochs))
        self.assertGreaterEqual(len(common_tows), 50)

        position_diffs: list[float] = []
        for tow in common_tows:
            x, y, z, status = cpp_positions[tow]
            self.assertEqual(status, 1)
            taroz_row = taroz_epochs[tow]
            tx = finite_float(taroz_row["fgo_x_m"])
            ty = finite_float(taroz_row["fgo_y_m"])
            tz = finite_float(taroz_row["fgo_z_m"])
            if tx is None or ty is None or tz is None:
                continue
            position_diffs.append(math.dist((x, y, z), (tx, ty, tz)))

        self.assertGreaterEqual(len(position_diffs), 50)
        self.assertLessEqual(sum(position_diffs) / len(position_diffs), 10.0)
        max_allowed_diff = 50.0 if len(position_diffs) > 1000 else 35.0
        self.assertLessEqual(max(position_diffs), max_allowed_diff)

    def test_final_position_file_matches_summary_contract(self) -> None:
        cpp_summary_path = CPP_DIR / "fgo_taroz_p_summary.json"
        cpp_pos_path = CPP_DIR / "fgo_taroz_p.pos"
        cpp_epoch_path = CPP_DIR / "fgo_taroz_p_epoch_debug.csv"
        self.require_dogfood_files(cpp_summary_path, cpp_pos_path, cpp_epoch_path)

        cpp_summary = read_json(cpp_summary_path)
        pos_rows = read_pos_rows(cpp_pos_path)
        with cpp_epoch_path.open(newline="") as handle:
            epoch_rows = list(csv.DictReader(handle))

        optimized_epochs = int(cpp_summary["optimized_epochs"])
        self.assertEqual(len(pos_rows), optimized_epochs)
        self.assertEqual(len(epoch_rows), optimized_epochs)
        self.assertEqual(int(cpp_summary["input_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_solutions"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["seed_matched_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["fixed_solutions"]), 0)
        self.assertEqual(int(cpp_summary["float_solutions"]), 0)

        epoch_indices = [int(float(row["epoch_index"])) for row in epoch_rows]
        self.assertEqual(epoch_indices, list(range(optimized_epochs)))
        pos_tows = [round(float(row["gps_tow"])) for row in pos_rows]
        epoch_tows = [round(float(row["gps_tow"])) for row in epoch_rows]
        self.assertEqual(pos_tows, epoch_tows)
        self.assertEqual(len(pos_tows), len(set(pos_tows)))
        self.assertTrue(
            all(current < following for current, following in zip(pos_tows, pos_tows[1:]))
        )
        if optimized_epochs in {80, 1141}:
            self.assertEqual(pos_tows[0], 553950)
            self.assertEqual(pos_tows[-1], 553950 + optimized_epochs - 1)
            self.assertEqual(
                Counter(
                    following - current
                    for current, following in zip(pos_tows, pos_tows[1:])
                ),
                Counter({1: optimized_epochs - 1}),
            )

        status_counts = Counter(int(float(row["status"])) for row in pos_rows)
        self.assertEqual(status_counts, Counter({1: optimized_epochs}))
        finite_pos_columns = ("x_m", "y_m", "z_m", "lat_deg", "lon_deg", "height_m", "pdop")
        finite_epoch_columns = (
            "position_x_m",
            "position_y_m",
            "position_z_m",
            "seed_position_x_m",
            "seed_position_y_m",
            "seed_position_z_m",
            "velocity_x_mps",
            "velocity_y_mps",
            "velocity_z_mps",
        )
        for pos, epoch in zip(pos_rows, epoch_rows):
            self.assertEqual(int(pos["gps_week"]), 2323)
            self.assertEqual(int(epoch["gps_week"]), 2323)
            self.assertEqual(int(float(pos["status"])), int(float(epoch["status"])))
            self.assertEqual(int(float(pos["fixed_ambiguities"])), int(epoch["num_fixed_ambiguities"]))
            self.assertEqual(int(float(pos["iterations"])), int(cpp_summary["iterations"]))
            self.assertGreaterEqual(
                int(float(pos["satellites"])),
                int(cpp_summary["min_satellites_per_epoch"]),
            )
            self.assertAlmostEqual(float(pos["x_m"]), float(epoch["position_x_m"]), places=6)
            self.assertAlmostEqual(float(pos["y_m"]), float(epoch["position_y_m"]), places=6)
            self.assertAlmostEqual(float(pos["z_m"]), float(epoch["position_z_m"]), places=6)
            self.assertAlmostEqual(float(pos["ratio"]), float(epoch["ratio"]), places=6)
            for column in finite_pos_columns:
                self.assertTrue(math.isfinite(float(pos[column])), column)
            for column in finite_epoch_columns:
                self.assertTrue(math.isfinite(float(epoch[column])), column)

    def test_graph_and_factor_counts_are_consistent(self) -> None:
        cpp_summary_path = CPP_DIR / "fgo_taroz_p_summary.json"
        cpp_pos_path = CPP_DIR / "fgo_taroz_p.pos"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(
            cpp_summary_path,
            cpp_pos_path,
            taroz_graph_path,
            taroz_sat_path,
        )

        cpp = read_json(cpp_summary_path)
        with taroz_graph_path.open(newline="") as handle:
            taroz = next(csv.DictReader(handle))
        cpp_tows = set(read_pos_by_tow(cpp_pos_path))

        self.assertEqual(cpp["preset"], "taroz-p")
        self.assertEqual(cpp["backend"], "eigen")
        self.assertFalse(cpp["use_spp_seed"])
        self.assertTrue(cpp["use_pseudorange_factors"])
        self.assertEqual(cpp["tdcp_factors"], 0)
        self.assertEqual(cpp["carrier_phase_factors"], 0)
        self.assertEqual(cpp["double_difference_pseudorange_factors"], 0)
        self.assertEqual(cpp["double_difference_carrier_factors"], 0)

        valid_pseudorange_rows = count_valid_pseudorange_rows(taroz_sat_path, cpp_tows)
        self.assertGreaterEqual(valid_pseudorange_rows, cpp["pseudorange_factors"])
        self.assertLessEqual(
            abs(valid_pseudorange_rows - cpp["pseudorange_factors"]) /
            max(1, valid_pseudorange_rows),
            0.02,
        )

        taroz_graph_factors = int(float(taroz["graph_factors"]))
        taroz_graph_values = int(float(taroz["graph_values"]))
        self.assertGreater(taroz_graph_factors, valid_pseudorange_rows)
        self.assertGreater(taroz_graph_values, int(cpp["optimized_epochs"]))
        if (
            cpp.get("debug_problem_only") is not True and
            int(cpp["optimized_epochs"]) == int(float(taroz["n"]))
        ):
            self.assertLessEqual(
                abs(float(cpp["final_cost"]) - float(taroz["final_cost"])) /
                max(1.0, abs(float(taroz["final_cost"]))),
                0.50,
            )

    def test_raw_pseudorange_factors_match_taroz_p_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "fgo_taroz_p_summary.json"
        cpp_pos_path = CPP_DIR / "fgo_taroz_p.pos"
        cpp_factor_path = CPP_DIR / "fgo_taroz_p_factor_debug.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(
            cpp_summary_path,
            cpp_pos_path,
            cpp_factor_path,
            taroz_sat_path,
        )

        cpp_summary = read_json(cpp_summary_path)
        cpp_tows = set(read_pos_by_tow(cpp_pos_path))
        cpp_factors = read_raw_factor_debug_by_key(cpp_factor_path)
        taroz_factors = read_valid_taroz_p_sat_rows_by_key(taroz_sat_path, cpp_tows)

        self.assertEqual(len(cpp_factors), cpp_summary["pseudorange_factors"])
        self.assertGreaterEqual(len(cpp_factors), 2400)
        self.assertEqual(set(cpp_factors), set(taroz_factors))

        keys = sorted(cpp_factors)
        self.assertLessEqual(
            max_abs_diff(
                keys,
                cpp_factors,
                taroz_factors,
                "elevation_deg",
                "elevation_deg",
            ),
            1e-3,
        )
        self.assertLessEqual(
            max_abs_diff(keys, cpp_factors, taroz_factors, "sigma_p_m", "sigma_p_m"),
            1e-4,
        )
        self.assertLessEqual(
            mean_abs_diff(keys, cpp_factors, taroz_factors, "initial_res_pc_m", "res_pc_m"),
            6.0,
        )
        self.assertLessEqual(
            max_abs_diff(keys, cpp_factors, taroz_factors, "initial_res_pc_m", "res_pc_m"),
            10.0,
        )
        for column in ("los_x", "los_y", "los_z"):
            self.assertLessEqual(
                max_abs_diff(keys, cpp_factors, taroz_factors, column, column),
                1e-5,
            )


if __name__ == "__main__":
    unittest.main()
