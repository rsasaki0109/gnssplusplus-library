#!/usr/bin/env python3
"""Optional parity checks against taroz MATLAB position/velocity PDC dumps."""

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
    "GNSSPP_TAROZ_POS_VEL_PDC_CPP_DIR",
    "output/dogfood/taroz_pos_vel_pdc_dogfood_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_POS_VEL_PDC_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pos_vel_pdc_debug",
)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT_DIR))
    except ValueError:
        return str(path)


def read_json(path: Path) -> dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def read_csv_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def read_epochs_by_tow(path: Path) -> dict[int, dict[str, str]]:
    return {
        round(float(row["gps_tow"])): row
        for row in read_csv_rows(path)
    }


def read_epochs_by_epoch(path: Path) -> dict[int, dict[str, str]]:
    return {
        int(float(row["epoch"])): row
        for row in read_csv_rows(path)
    }


def read_graph(path: Path) -> dict[str, str]:
    with path.open(newline="") as handle:
        return next(csv.DictReader(handle))


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def max_abs_diff(
    keys: list[tuple[str, int, str]],
    left: dict[tuple[str, int, str], dict[str, str]],
    right: dict[tuple[str, int, str], dict[str, str]],
    left_column: str,
    right_column: str,
) -> float:
    return max(
        abs(float(left[key][left_column]) - float(right[key][right_column]))
        for key in keys
    )


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


def read_cpp_factor_rows(path: Path) -> dict[tuple[str, int, str], dict[str, str]]:
    rows: dict[tuple[str, int, str], dict[str, str]] = {}
    for row in read_csv_rows(path):
        key = (row["factor_type"], round(float(row["gps_tow"])), row["satellite"])
        rows[key] = row
    return rows


def read_taroz_factor_rows(
    satellite_path: Path,
    tdcp_path: Path,
    allowed_tows: set[int] | None = None,
) -> dict[tuple[str, int, str], dict[str, str]]:
    rows: dict[tuple[str, int, str], dict[str, str]] = {}
    for row in read_csv_rows(satellite_path):
        tow = round(float(row["gps_tow"]))
        if allowed_tows is not None and tow not in allowed_tows:
            continue
        if row.get("valid_pseudorange", "").lower() in {"1", "true"}:
            rows[("P", tow, row["satellite"])] = row
        if row.get("valid_doppler", "").lower() in {"1", "true"}:
            rows[("D", tow, row["satellite"])] = row
    for row in read_csv_rows(tdcp_path):
        tow = round(float(row["gps_tow"]))
        if allowed_tows is not None and tow not in allowed_tows:
            continue
        if row.get("valid_tdcp", "").lower() in {"1", "true"}:
            rows[("C", tow, row["satellite"])] = row
    return rows


def clock_group_from_system(system: str) -> int:
    return {
        "SYS_GPS": 0,
        "SYS_GLO": 1,
        "SYS_GAL": 2,
        "SYS_QZS": 3,
        "SYS_CMP": 4,
    }.get(system, 0)


def taroz_position_delta(epoch: dict[str, str]) -> list[float]:
    return [
        float(epoch[f"fgo_{axis}_m"]) - float(epoch[f"spp_{axis}_m"])
        for axis in "xyz"
    ]


def taroz_pseudorange_prediction(
    epoch: dict[str, str],
    factor: dict[str, str],
) -> float:
    delta_position = taroz_position_delta(epoch)
    los = [float(factor[f"los_{axis}"]) for axis in "xyz"]
    clock_columns = [
        "fgo_c_gps_m",
        "fgo_c_glo_m",
        "fgo_c_gal_m",
        "fgo_c_qzs_m",
        "fgo_c_bds_m",
    ]
    group = clock_group_from_system(factor["system"])
    clock = float(epoch["fgo_c_gps_m"])
    if group != 0:
        clock += float(epoch[clock_columns[group]])
    return sum(l * dx for l, dx in zip(los, delta_position)) + clock


def taroz_doppler_prediction(
    epoch: dict[str, str],
    factor: dict[str, str],
) -> float:
    velocity = [float(epoch[f"fgo_v{axis}_mps"]) for axis in "xyz"]
    los = [float(factor[f"los_{axis}"]) for axis in "xyz"]
    return sum(l * v for l, v in zip(los, velocity)) + float(epoch["fgo_clock_drift_mps"])


def taroz_tdcp_prediction(
    current_epoch: dict[str, str],
    previous_epoch: dict[str, str],
    factor: dict[str, str],
) -> float:
    current_delta = taroz_position_delta(current_epoch)
    previous_delta = taroz_position_delta(previous_epoch)
    los = [float(factor[f"los_{axis}"]) for axis in "xyz"]
    return (
        sum(
            l * (current - previous)
            for l, current, previous in zip(los, current_delta, previous_delta)
        )
        + float(current_epoch["fgo_c_gps_m"])
        - float(previous_epoch["fgo_c_gps_m"])
    )


class TarozPosVelPDCInternalParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [display_path(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest(
                "missing optional taroz position/velocity PDC dogfood files: "
                + ", ".join(missing)
            )

    def test_graph_and_factor_counts_match_taroz_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        taroz_tdcp_path = MATLAB_DIR / "per_tdcp_detail.csv"
        self.require_dogfood_files(
            cpp_summary_path,
            cpp_graph_path,
            cpp_factor_path,
            cpp_epoch_path,
            taroz_graph_path,
            taroz_sat_path,
            taroz_tdcp_path,
        )

        cpp_summary = read_json(cpp_summary_path)
        cpp_graph = read_graph(cpp_graph_path)
        taroz_graph = read_graph(taroz_graph_path)
        cpp_factors = read_cpp_factor_rows(cpp_factor_path)
        cpp_tows = set(read_epochs_by_tow(cpp_epoch_path))
        taroz_factors = read_taroz_factor_rows(taroz_sat_path, taroz_tdcp_path, cpp_tows)
        p_keys = [key for key in cpp_factors if key[0] == "P"]
        d_keys = [key for key in cpp_factors if key[0] == "D"]
        c_keys = [key for key in cpp_factors if key[0] == "C"]
        optimized_epochs = int(cpp_summary["optimized_epochs"])
        taroz_epochs = int(float(taroz_graph["n"]))

        self.assertEqual(cpp_summary["preset"], "taroz-pdc")
        self.assertEqual(cpp_summary["backend"], "eigen")
        self.assertEqual(optimized_epochs, len(cpp_tows))
        self.assertLessEqual(optimized_epochs, taroz_epochs)
        self.assertEqual(cpp_summary["valid_position_epochs"], optimized_epochs)
        self.assertEqual(cpp_summary["valid_velocity_epochs"], optimized_epochs)
        self.assertEqual(cpp_summary["pseudorange_factors"], len(p_keys))
        self.assertEqual(cpp_summary["doppler_factors"], len(d_keys))
        self.assertEqual(cpp_summary["tdcp_factors"], len(c_keys))
        self.assertGreater(len(p_keys), 0)
        self.assertGreater(len(d_keys), 0)
        self.assertGreater(len(c_keys), 0)
        self.assertEqual(set(cpp_factors), set(taroz_factors))
        self.assertEqual(int(float(cpp_graph["graph_values"])), 4 * optimized_epochs)
        self.assertEqual(int(cpp_summary["graph_values"]), int(float(cpp_graph["graph_values"])))
        self.assertEqual(int(cpp_summary["graph_factors"]), int(float(cpp_graph["graph_factors"])))
        if optimized_epochs == taroz_epochs:
            self.assertEqual(len(p_keys), 24092)
            self.assertEqual(len(d_keys), 24092)
            self.assertEqual(len(c_keys), 21832)
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
                0.35,
            )

    def test_final_epoch_state_file_matches_summary_contract(self) -> None:
        cpp_summary_path = CPP_DIR / "summary.json"
        cpp_graph_path = CPP_DIR / "graph_detail.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(cpp_summary_path, cpp_graph_path, cpp_epoch_path)

        cpp_summary = read_json(cpp_summary_path)
        cpp_graph = read_graph(cpp_graph_path)
        rows = read_csv_rows(cpp_epoch_path)
        optimized_epochs = int(cpp_summary["optimized_epochs"])
        self.assertEqual(len(rows), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_position_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["valid_velocity_epochs"]), optimized_epochs)
        self.assertEqual(int(cpp_summary["graph_values"]), 4 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["graph_values"])), 4 * optimized_epochs)
        self.assertEqual(int(float(cpp_graph["n"])), optimized_epochs)
        self.assertEqual(int(float(cpp_graph["valid_position_epochs"])), optimized_epochs)
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
            "fgo_vx_mps",
            "fgo_vy_mps",
            "fgo_vz_mps",
            "fgo_c_gps_m",
            "fgo_c_glo_m",
            "fgo_c_gal_m",
            "fgo_c_qzs_m",
            "fgo_c_bds_m",
            "fgo_clock_drift_mps",
        )
        for row in rows:
            self.assertEqual(int(row["gps_week"]), 2323)
            self.assertIn(row["clock_jump"], {"0", "1"})
            for column in finite_columns:
                self.assertTrue(math.isfinite(float(row[column])), column)

    def test_raw_factors_match_taroz_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        taroz_tdcp_path = MATLAB_DIR / "per_tdcp_detail.csv"
        self.require_dogfood_files(cpp_factor_path, cpp_epoch_path, taroz_sat_path, taroz_tdcp_path)

        cpp_factors = read_cpp_factor_rows(cpp_factor_path)
        cpp_tows = set(read_epochs_by_tow(cpp_epoch_path))
        taroz_factors = read_taroz_factor_rows(taroz_sat_path, taroz_tdcp_path, cpp_tows)
        self.assertEqual(set(cpp_factors), set(taroz_factors))

        p_keys = sorted(key for key in cpp_factors if key[0] == "P")
        d_keys = sorted(key for key in cpp_factors if key[0] == "D")
        c_keys = sorted(key for key in cpp_factors if key[0] == "C")
        self.assertGreater(len(p_keys), 0)
        self.assertGreater(len(d_keys), 0)
        self.assertGreater(len(c_keys), 0)
        if len(cpp_tows) == 1141:
            self.assertEqual(len(p_keys), 24092)
            self.assertEqual(len(d_keys), 24092)
            self.assertEqual(len(c_keys), 21832)

        self.assertLessEqual(max_abs_diff(p_keys, cpp_factors, taroz_factors, "elevation_deg", "elevation_deg"), 1e-3)
        self.assertLessEqual(max_abs_diff(p_keys, cpp_factors, taroz_factors, "sigma_p_m", "sigma_p_m"), 1e-4)
        self.assertLessEqual(mean_abs_diff(p_keys, cpp_factors, taroz_factors, "res_pc_m", "res_pc_m"), 6.0)
        self.assertLessEqual(max_abs_diff(p_keys, cpp_factors, taroz_factors, "res_pc_m", "res_pc_m"), 10.0)

        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "elevation_deg", "elevation_deg"), 1e-4)
        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "sigma_d_mps", "sigma_d_mps"), 1e-6)
        self.assertLessEqual(mean_abs_diff(d_keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 1e-3)
        self.assertLessEqual(max_abs_diff(d_keys, cpp_factors, taroz_factors, "res_d_mps", "res_d_mps"), 0.02)
        for axis in "xyz":
            self.assertLessEqual(
                max_abs_diff(d_keys, cpp_factors, taroz_factors, f"los_{axis}", f"los_{axis}"),
                1e-6,
            )

        self.assertLessEqual(max_abs_diff(c_keys, cpp_factors, taroz_factors, "sigma_c_m", "sigma_c_m"), 1e-5)
        tdcp_diffs = [
            abs(float(cpp_factors[key]["res_tdcp_m"]) - float(taroz_factors[key]["tdcp_m"]))
            for key in c_keys
        ]
        self.assertLessEqual(statistics.mean(tdcp_diffs), 0.003)
        self.assertLessEqual(percentile(tdcp_diffs, 0.95), 0.01)
        self.assertLessEqual(max(tdcp_diffs), 4.0)
        for axis in "xyz":
            self.assertLessEqual(
                max_abs_diff(c_keys, cpp_factors, taroz_factors, f"los_{axis}", f"los_{axis}"),
                1e-6,
            )

    def test_final_state_matches_taroz_dump(self) -> None:
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(cpp_epoch_path, taroz_epoch_path)

        cpp_epochs = read_epochs_by_tow(cpp_epoch_path)
        taroz_epochs = read_epochs_by_tow(taroz_epoch_path)
        common_tows = sorted(set(cpp_epochs) & set(taroz_epochs))
        self.assertEqual(len(common_tows), len(cpp_epochs))
        self.assertGreater(len(common_tows), 0)
        if len(cpp_epochs) == len(taroz_epochs):
            self.assertEqual(len(common_tows), 1141)

        position_diffs: list[float] = []
        velocity_diffs: list[float] = []
        drift_diffs: list[float] = []
        clock_diffs: list[float] = []
        for tow in common_tows:
            cpp = cpp_epochs[tow]
            taroz = taroz_epochs[tow]
            position_diffs.append(
                math.dist(
                    tuple(float(cpp[f"fgo_{axis}_m"]) for axis in "xyz"),
                    tuple(float(taroz[f"fgo_{axis}_m"]) for axis in "xyz"),
                )
            )
            velocity_diffs.append(
                math.dist(
                    tuple(float(cpp[f"fgo_v{axis}_mps"]) for axis in "xyz"),
                    tuple(float(taroz[f"fgo_v{axis}_mps"]) for axis in "xyz"),
                )
            )
            drift_diffs.append(
                abs(float(cpp["fgo_clock_drift_mps"]) - float(taroz["fgo_clock_drift_mps"]))
            )
            for column in (
                "fgo_c_gps_m",
                "fgo_c_glo_m",
                "fgo_c_gal_m",
                "fgo_c_qzs_m",
                "fgo_c_bds_m",
            ):
                clock_diffs.append(abs(float(cpp[column]) - float(taroz[column])))

        self.assertLessEqual(statistics.mean(position_diffs), 3.0)
        self.assertLessEqual(percentile(position_diffs, 0.95), 4.0)
        self.assertLessEqual(max(position_diffs), 5.5)
        self.assertLessEqual(statistics.mean(velocity_diffs), 0.05)
        self.assertLessEqual(percentile(velocity_diffs, 0.95), 0.10)
        self.assertLessEqual(max(velocity_diffs), 3.5)
        self.assertLessEqual(statistics.mean(drift_diffs), 0.02)
        self.assertLessEqual(max(drift_diffs), 0.20)
        self.assertLessEqual(statistics.mean(clock_diffs), 3.0)
        self.assertLessEqual(max(clock_diffs), 8.0)

    def test_final_factor_residuals_match_taroz_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        cpp_epoch_path = CPP_DIR / "per_epoch_state.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        taroz_tdcp_path = MATLAB_DIR / "per_tdcp_detail.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(
            cpp_factor_path,
            cpp_epoch_path,
            taroz_sat_path,
            taroz_tdcp_path,
            taroz_epoch_path,
        )

        cpp_factors = read_cpp_factor_rows(cpp_factor_path)
        cpp_tows = set(read_epochs_by_tow(cpp_epoch_path))
        taroz_factors = read_taroz_factor_rows(taroz_sat_path, taroz_tdcp_path, cpp_tows)
        taroz_epochs_by_tow = read_epochs_by_tow(taroz_epoch_path)
        taroz_epochs_by_epoch = read_epochs_by_epoch(taroz_epoch_path)
        self.assertEqual(set(cpp_factors), set(taroz_factors))

        pseudorange_diffs: list[float] = []
        doppler_diffs: list[float] = []
        tdcp_diffs: list[float] = []
        for key in sorted(cpp_factors):
            factor_type, tow, _satellite = key
            cpp = cpp_factors[key]
            taroz = taroz_factors[key]
            if factor_type == "P":
                residual = taroz_pseudorange_prediction(taroz_epochs_by_tow[tow], taroz) - float(taroz["res_pc_m"])
                pseudorange_diffs.append(abs(float(cpp["final_residual_m"]) - residual))
            elif factor_type == "D":
                residual = taroz_doppler_prediction(taroz_epochs_by_tow[tow], taroz) - float(taroz["res_d_mps"])
                doppler_diffs.append(abs(float(cpp["final_residual_m"]) - residual))
            else:
                current_epoch = taroz_epochs_by_epoch[int(float(taroz["epoch"]))]
                previous_epoch = taroz_epochs_by_epoch[int(float(taroz["previous_epoch"]))]
                residual = taroz_tdcp_prediction(current_epoch, previous_epoch, taroz) - float(taroz["tdcp_m"])
                tdcp_diffs.append(abs(float(cpp["final_residual_m"]) - residual))

        self.assertGreater(len(pseudorange_diffs), 0)
        self.assertGreater(len(doppler_diffs), 0)
        self.assertGreater(len(tdcp_diffs), 0)
        if len(cpp_tows) == 1141:
            self.assertEqual(len(pseudorange_diffs), 24092)
            self.assertEqual(len(doppler_diffs), 24092)
            self.assertEqual(len(tdcp_diffs), 21832)
        self.assertLessEqual(statistics.mean(pseudorange_diffs), 1.5)
        self.assertLessEqual(percentile(pseudorange_diffs, 0.95), 3.2)
        self.assertLessEqual(max(pseudorange_diffs), 4.0)
        self.assertLessEqual(statistics.mean(doppler_diffs), 0.002)
        self.assertLessEqual(percentile(doppler_diffs, 0.95), 0.005)
        self.assertLessEqual(max(doppler_diffs), 1.1)
        self.assertLessEqual(statistics.mean(tdcp_diffs), 0.002)
        self.assertLessEqual(percentile(tdcp_diffs, 0.95), 0.005)
        self.assertLessEqual(max(tdcp_diffs), 2.5)


if __name__ == "__main__":
    unittest.main()
