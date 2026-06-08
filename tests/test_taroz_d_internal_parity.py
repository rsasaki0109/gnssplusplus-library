#!/usr/bin/env python3
"""Optional parity checks against taroz MATLAB Doppler velocity dumps."""

from __future__ import annotations

import csv
from collections import Counter
import json
import math
import os
from pathlib import Path
import statistics
import unittest


ROOT_DIR = Path(__file__).resolve().parents[1]


def configured_path(env_name: str, default_relative_path: str) -> Path:
    configured = os.environ.get(env_name)
    if configured:
        path = Path(configured)
        return path if path.is_absolute() else ROOT_DIR / path
    return ROOT_DIR / default_relative_path


CPP_DIR = configured_path(
    "GNSSPP_TAROZ_D_CPP_DIR",
    "output/dogfood/taroz_d_dogfood_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_D_MATLAB_DIR",
    "output/dogfood/taroz_matlab_d_debug",
)


def read_json(path: Path) -> dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def read_csv_by_tow(path: Path) -> dict[int, dict[str, str]]:
    with path.open(newline="") as handle:
        return {
            round(float(row["gps_tow"])): row
            for row in csv.DictReader(handle)
        }


def read_cpp_factor_rows(path: Path) -> dict[tuple[int, str], dict[str, str]]:
    with path.open(newline="") as handle:
        return {
            (round(float(row["gps_tow"])), row["satellite"]): row
            for row in csv.DictReader(handle)
        }


def read_taroz_factor_rows(path: Path) -> dict[tuple[int, str], dict[str, str]]:
    rows: dict[tuple[int, str], dict[str, str]] = {}
    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("valid_doppler", "").lower() not in {"1", "true"}:
                continue
            rows[(round(float(row["gps_tow"])), row["satellite"])] = row
    return rows


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
    return statistics.mean(
        abs(float(left[key][left_column]) - float(right[key][right_column]))
        for key in keys
    )


class TarozDInternalParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [str(path.relative_to(ROOT_DIR)) for path in paths if not path.exists()]
        if missing:
            self.skipTest("missing optional taroz D dogfood files: " + ", ".join(missing))

    def test_graph_and_factor_counts_match_taroz_d_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(
            cpp_summary_path,
            cpp_graph_path,
            taroz_graph_path,
            taroz_sat_path,
        )

        cpp = read_json(cpp_summary_path)
        with cpp_graph_path.open(newline="") as handle:
            cpp_graph = next(csv.DictReader(handle))
        with taroz_graph_path.open(newline="") as handle:
            taroz_graph = next(csv.DictReader(handle))
        taroz_factors = read_taroz_factor_rows(taroz_sat_path)

        self.assertEqual(cpp["preset"], "taroz-d")
        self.assertEqual(cpp["backend"], "eigen")
        self.assertEqual(cpp["optimized_epochs"], int(float(taroz_graph["n"])))
        self.assertEqual(cpp["doppler_factors"], len(taroz_factors))
        self.assertEqual(int(float(cpp_graph["graph_factors"])), int(float(taroz_graph["graph_factors"])))
        self.assertEqual(int(float(cpp_graph["graph_values"])), int(float(taroz_graph["graph_values"])))
        self.assertLessEqual(
            abs(float(cpp_graph["initial_cost"]) - float(taroz_graph["initial_cost"])) /
            max(1.0, abs(float(taroz_graph["initial_cost"]))),
            1e-4,
        )
        self.assertLessEqual(
            abs(float(cpp_graph["final_cost"]) - float(taroz_graph["final_cost"])) /
            max(1.0, abs(float(taroz_graph["final_cost"]))),
            1e-3,
        )

    def test_final_epoch_velocity_file_matches_summary_contract(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_vel.csv"
        self.require_dogfood_files(cpp_summary_path, cpp_graph_path, cpp_epoch_path)

        cpp_summary = read_json(cpp_summary_path)
        with cpp_graph_path.open(newline="") as handle:
            cpp_graph = next(csv.DictReader(handle))
        with cpp_epoch_path.open(newline="") as handle:
            rows = list(csv.DictReader(handle))

        optimized_epochs = int(cpp_summary["optimized_epochs"])
        self.assertEqual(len(rows), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_velocity_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["graph_values"]), 2 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["graph_values"])), 2 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["n"])), optimized_epochs)
        self.assertEqual(int(float(cpp_graph["valid_velocity_epochs"])), optimized_epochs)
        self.assertEqual(int(cpp_summary["iterations"]), int(float(cpp_graph["iterations"])))
        self.assertEqual(float(cpp_summary["final_cost"]), float(cpp_graph["final_cost"]))
        self.assertEqual(float(cpp_graph["final_cost"]), float(cpp_graph["optimizer_error"]))

        epoch_indices = [int(float(row["epoch"])) for row in rows]
        self.assertEqual(epoch_indices, list(range(1, optimized_epochs + 1)))
        gps_tows = [round(float(row["gps_tow"])) for row in rows]
        self.assertEqual(len(gps_tows), len(set(gps_tows)))
        self.assertTrue(
            all(current < following for current, following in zip(gps_tows, gps_tows[1:]))
        )
        if optimized_epochs == 1141:
            self.assertEqual(gps_tows[0], 553950)
            self.assertEqual(gps_tows[-1], 555090)
            self.assertEqual(
                Counter(
                    following - current
                    for current, following in zip(gps_tows, gps_tows[1:])
                ),
                Counter({1: 1140}),
            )

        finite_columns = (
            "spp_x_m",
            "spp_y_m",
            "spp_z_m",
            "fgo_vx_mps",
            "fgo_vy_mps",
            "fgo_vz_mps",
            "fgo_clock_drift_mps",
        )
        for row in rows:
            self.assertEqual(int(row["gps_week"]), 2323)
            for column in finite_columns:
                self.assertTrue(math.isfinite(float(row[column])), column)

    def test_raw_doppler_factors_match_taroz_d_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(cpp_factor_path, taroz_sat_path)

        cpp_factors = read_cpp_factor_rows(cpp_factor_path)
        taroz_factors = read_taroz_factor_rows(taroz_sat_path)

        self.assertEqual(set(cpp_factors), set(taroz_factors))
        keys = sorted(cpp_factors)
        self.assertEqual(len(keys), 24092)
        self.assertLessEqual(max_abs_diff(keys, cpp_factors, taroz_factors, "elevation_deg", "elevation_deg"), 1e-4)
        self.assertLessEqual(max_abs_diff(keys, cpp_factors, taroz_factors, "sigma_d_mps", "sigma_d_mps"), 1e-6)
        self.assertLessEqual(mean_abs_diff(keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 1e-3)
        self.assertLessEqual(max_abs_diff(keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 0.02)
        for axis in ("x", "y", "z"):
            self.assertLessEqual(
                max_abs_diff(keys, cpp_factors, taroz_factors, f"los_{axis}", f"los_{axis}"),
                1e-6,
            )

    def test_final_velocity_matches_taroz_d_dump(self) -> None:
        cpp_epoch_path = CPP_DIR / "per_epoch_vel.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_vel.csv"
        self.require_dogfood_files(cpp_epoch_path, taroz_epoch_path)

        cpp_epochs = read_csv_by_tow(cpp_epoch_path)
        taroz_epochs = read_csv_by_tow(taroz_epoch_path)
        common_tows = sorted(set(cpp_epochs) & set(taroz_epochs))
        self.assertEqual(len(common_tows), 1141)

        velocity_diffs: list[float] = []
        clock_diffs: list[float] = []
        for tow in common_tows:
            cpp = cpp_epochs[tow]
            taroz = taroz_epochs[tow]
            cpp_velocity = (
                float(cpp["fgo_vx_mps"]),
                float(cpp["fgo_vy_mps"]),
                float(cpp["fgo_vz_mps"]),
            )
            taroz_velocity = (
                float(taroz["fgo_vx_mps"]),
                float(taroz["fgo_vy_mps"]),
                float(taroz["fgo_vz_mps"]),
            )
            velocity_diffs.append(math.dist(cpp_velocity, taroz_velocity))
            clock_diffs.append(
                abs(float(cpp["fgo_clock_drift_mps"]) - float(taroz["fgo_clock_drift_mps"]))
            )

        self.assertLessEqual(statistics.mean(velocity_diffs), 0.01)
        self.assertLessEqual(max(velocity_diffs), 0.05)
        self.assertLessEqual(statistics.mean(clock_diffs), 0.01)
        self.assertLessEqual(max(clock_diffs), 0.05)


if __name__ == "__main__":
    unittest.main()
