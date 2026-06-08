#!/usr/bin/env python3
"""Optional parity checks against taroz MATLAB position PD dumps."""

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
    "GNSSPP_TAROZ_POS_PD_CPP_DIR",
    "output/dogfood/taroz_pos_pd_dogfood_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_POS_PD_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pos_pd_debug",
)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT_DIR))
    except ValueError:
        return str(path)


def read_json(path: Path) -> dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def read_graph(path: Path) -> dict[str, str]:
    with path.open(newline="") as handle:
        return next(csv.DictReader(handle))


def read_epochs(path: Path) -> dict[int, dict[str, str]]:
    return {
        round(float(row["gps_tow"])): row
        for row in read_rows(path)
    }


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def read_cpp_factors(path: Path) -> dict[tuple[str, int, str], dict[str, str]]:
    factors: dict[tuple[str, int, str], dict[str, str]] = {}
    for row in read_rows(path):
        epoch = int(row["epoch_index"]) + 1
        factors[(row["factor_type"], epoch, row["satellite"])] = row
    return factors


def read_taroz_factors(path_p: Path, path_d: Path) -> dict[tuple[str, int, str], dict[str, str]]:
    factors: dict[tuple[str, int, str], dict[str, str]] = {}
    for row in read_rows(path_p):
        if row.get("valid_pseudorange", "").lower() in {"1", "true"}:
            factors[("P", int(float(row["epoch"])), row["satellite"])] = row
    for row in read_rows(path_d):
        if row.get("valid_doppler", "").lower() in {"1", "true"}:
            factors[("D", int(float(row["epoch"])), row["satellite"])] = row
    return factors


def max_abs_diff(
    keys: list[tuple[str, int, str]],
    left: dict[tuple[str, int, str], dict[str, str]],
    right: dict[tuple[str, int, str], dict[str, str]],
    left_column: str,
    right_column: str,
) -> float:
    return max(abs(float(left[key][left_column]) - float(right[key][right_column])) for key in keys)


def mean_abs_diff(
    keys: list[tuple[str, int, str]],
    left: dict[tuple[str, int, str], dict[str, str]],
    right: dict[tuple[str, int, str], dict[str, str]],
    left_column: str,
    right_column: str,
) -> float:
    return statistics.mean(
        abs(float(left[key][left_column]) - float(right[key][right_column]))
        for key in keys
    )


class TarozPosPDInternalParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [display_path(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest("missing optional taroz position PD dogfood files: " + ", ".join(missing))

    def test_graph_and_factor_counts_match_taroz_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        taroz_p_path = MATLAB_DIR / "per_sat_detail.csv"
        taroz_d_path = MATLAB_DIR / "per_doppler_detail.csv"
        self.require_dogfood_files(
            cpp_summary_path,
            cpp_graph_path,
            cpp_factor_path,
            taroz_graph_path,
            taroz_p_path,
            taroz_d_path,
        )

        cpp_summary = read_json(cpp_summary_path)
        cpp_graph = read_graph(cpp_graph_path)
        taroz_graph = read_graph(taroz_graph_path)
        cpp_factors = read_cpp_factors(cpp_factor_path)
        taroz_factors = read_taroz_factors(taroz_p_path, taroz_d_path)
        p_keys = [key for key in cpp_factors if key[0] == "P"]
        d_keys = [key for key in cpp_factors if key[0] == "D"]

        self.assertEqual(cpp_summary["preset"], "taroz-pos-pd")
        self.assertEqual(cpp_summary["backend"], "eigen")
        self.assertEqual(cpp_summary["optimized_epochs"], int(float(taroz_graph["n"])))
        self.assertEqual(cpp_summary["pseudorange_factors"], len(p_keys))
        self.assertEqual(cpp_summary["doppler_factors"], len(d_keys))
        self.assertEqual(len(p_keys), 24092)
        self.assertEqual(len(d_keys), 29371)
        self.assertEqual(set(cpp_factors), set(taroz_factors))
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
            0.15,
        )

    def test_final_epoch_state_file_matches_summary_contract(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(cpp_summary_path, cpp_graph_path, cpp_epoch_path)

        cpp_summary = read_json(cpp_summary_path)
        cpp_graph = read_graph(cpp_graph_path)
        rows = read_rows(cpp_epoch_path)
        optimized_epochs = int(cpp_summary["optimized_epochs"])
        self.assertEqual(len(rows), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_position_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_clock_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["graph_values"]), 2 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["graph_values"])), 2 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["n"])), optimized_epochs)
        self.assertEqual(int(float(cpp_graph["valid_position_epochs"])), optimized_epochs)
        self.assertEqual(int(float(cpp_graph["valid_clock_epochs"])), optimized_epochs)
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
            self.assertEqual(
                Counter(row["clock_jump"] for row in rows),
                Counter({"0": 1140, "1": 1}),
            )

        finite_columns = (
            "spp_x_m",
            "spp_y_m",
            "spp_z_m",
            "fgo_x_m",
            "fgo_y_m",
            "fgo_z_m",
            "fgo_c_gps_m",
            "fgo_c_glo_m",
            "fgo_c_gal_m",
            "fgo_c_qzs_m",
            "fgo_c_bds_m",
        )
        for row in rows:
            self.assertEqual(int(row["gps_week"]), 2323)
            self.assertIn(row["clock_jump"], {"0", "1"})
            for column in finite_columns:
                self.assertTrue(math.isfinite(float(row[column])), column)

    def test_raw_factors_match_taroz_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        taroz_p_path = MATLAB_DIR / "per_sat_detail.csv"
        taroz_d_path = MATLAB_DIR / "per_doppler_detail.csv"
        self.require_dogfood_files(cpp_factor_path, taroz_p_path, taroz_d_path)

        cpp_factors = read_cpp_factors(cpp_factor_path)
        taroz_factors = read_taroz_factors(taroz_p_path, taroz_d_path)
        self.assertEqual(set(cpp_factors), set(taroz_factors))

        p_keys = sorted(key for key in cpp_factors if key[0] == "P")
        d_keys = sorted(key for key in cpp_factors if key[0] == "D")
        self.assertEqual(len(p_keys), 24092)
        self.assertEqual(len(d_keys), 29371)

        self.assertLessEqual(max_abs_diff(p_keys, cpp_factors, taroz_factors, "sigma_p_m", "sigma_p_m"), 1e-4)
        self.assertLessEqual(mean_abs_diff(p_keys, cpp_factors, taroz_factors, "res_pc_m", "res_pc_m"), 6.0)
        self.assertLessEqual(max_abs_diff(p_keys, cpp_factors, taroz_factors, "res_pc_m", "res_pc_m"), 10.0)

        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "sigma_d_mps", "sigma_d_mps"), 1e-5)
        self.assertLessEqual(mean_abs_diff(d_keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 1e-3)
        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 0.01)
        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "midpoint_gps_tow", "midpoint_gps_tow"), 1e-9)
        for axis in "xyz":
            self.assertLessEqual(
                max_abs_diff(d_keys, cpp_factors, taroz_factors, f"los_{axis}", f"los_{axis}"),
                1e-6,
            )

    def test_final_position_matches_taroz_dump(self) -> None:
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(cpp_epoch_path, taroz_epoch_path)

        cpp_epochs = read_epochs(cpp_epoch_path)
        taroz_epochs = read_epochs(taroz_epoch_path)
        common_tows = sorted(set(cpp_epochs) & set(taroz_epochs))
        self.assertEqual(len(common_tows), 1141)

        position_diffs: list[float] = []
        clock_diffs: list[float] = []
        for tow in common_tows:
            cpp = cpp_epochs[tow]
            taroz = taroz_epochs[tow]
            taroz_position = tuple(float(taroz[f"fgo_{axis}_m"]) for axis in "xyz")
            if all(math.isfinite(value) for value in taroz_position):
                position_diffs.append(
                    math.dist(
                        tuple(float(cpp[f"fgo_{axis}_m"]) for axis in "xyz"),
                        taroz_position,
                    )
                )
            for column in (
                "fgo_c_gps_m",
                "fgo_c_glo_m",
                "fgo_c_gal_m",
                "fgo_c_qzs_m",
                "fgo_c_bds_m",
            ):
                clock_diffs.append(abs(float(cpp[column]) - float(taroz[column])))

        self.assertEqual(len(position_diffs), 1129)
        self.assertLessEqual(statistics.mean(position_diffs), 3.0)
        self.assertLessEqual(percentile(position_diffs, 0.95), 4.0)
        self.assertLessEqual(max(position_diffs), 5.0)
        self.assertLessEqual(statistics.mean(clock_diffs), 3.0)
        self.assertLessEqual(max(clock_diffs), 8.0)


if __name__ == "__main__":
    unittest.main()
