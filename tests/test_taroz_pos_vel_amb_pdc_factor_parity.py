#!/usr/bin/env python3
"""Optional raw-factor parity checks for taroz position/velocity ambiguity PDC."""

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
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_DIR",
    "output/dogfood/taroz_pos_vel_amb_pdc_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_debug",
)
CPP_OPT_POS = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_POS",
    "output/dogfood/taroz_pos_vel_amb_pdc_current/opt.pos",
)
CPP_OPT_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_EPOCH_DEBUG",
    "output/dogfood/taroz_pos_vel_amb_pdc_current/epoch_debug.csv",
)
CPP_OPT_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_LAMBDA_DEBUG",
    "output/dogfood/taroz_pos_vel_amb_pdc_current/lambda_debug.csv",
)
CPP_OPT_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_CPP_OPT_COST_TRACE",
    "output/dogfood/taroz_pos_vel_amb_pdc_current/cost_trace.csv",
)
MATLAB_OPTIMIZER_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_debug/optimizer_cost_trace.csv",
)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT_DIR))
    except ValueError:
        return str(path)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, object]:
    with path.open() as handle:
        return json.load(handle)


def read_pos_rows(path: Path) -> dict[tuple[int, int], dict[str, float | int]]:
    rows: dict[tuple[int, int], dict[str, float | int]] = {}
    with path.open() as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            if len(parts) < 14:
                continue
            key = (int(float(parts[0])), round(float(parts[1])))
            rows[key] = {
                "x": float(parts[2]),
                "y": float(parts[3]),
                "z": float(parts[4]),
                "status": int(float(parts[8])),
                "satellites": int(float(parts[9])),
                "ratio": float(parts[11]),
                "fixed_ambiguities": int(float(parts[12])),
                "iterations": int(float(parts[13])),
            }
    return rows


def keyed_epoch_rows(path: Path) -> dict[tuple[int, int], dict[str, str]]:
    return {
        (int(float(row["gps_week"])), round(float(row["gps_tow"]))): row
        for row in read_rows(path)
    }


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    return ordered[int(fraction * (len(ordered) - 1))]


def finite(value: str) -> bool:
    try:
        return math.isfinite(float(value))
    except ValueError:
        return False


def relative_error(actual: float, expected: float) -> float:
    return abs(actual - expected) / max(1.0, abs(expected))


def keyed_cpp_rows(path: Path) -> dict[tuple[int, str, str], dict[str, str]]:
    return {
        (round(float(row["gps_tow"])), row["satellite"], row["reference"]): row
        for row in read_rows(path)
    }


def keyed_taroz_rows(path: Path) -> dict[tuple[int, str, str], dict[str, str]]:
    rows: dict[tuple[int, str, str], dict[str, str]] = {}
    for row in read_rows(path):
        if row.get("valid_pseudorange_dd", "").lower() in {"1", "true"}:
            rows[(round(float(row["gps_tow"])), row["satellite"], row["reference"])] = row
    return rows


def keyed_valid_rows(
    path: Path,
    valid_column: str,
) -> dict[tuple[int, str, str], dict[str, str]]:
    rows: dict[tuple[int, str, str], dict[str, str]] = {}
    for row in read_rows(path):
        if row.get(valid_column, "").lower() in {"1", "true"}:
            rows[(round(float(row["gps_tow"])), row["satellite"], row["reference"])] = row
    return rows


def keyed_lambda_rows(
    path: Path,
) -> dict[tuple[int, int, int, int, str, str], dict[str, str]]:
    return {
        (
            int(float(row["gps_week"])),
            round(float(row["gps_tow"])),
            int(row["row"]),
            int(row["col"]),
            row["satellite"],
            row["other_satellite"],
        ): row
        for row in read_rows(path)
    }


def grouped_lambda_rows(
    path: Path,
) -> dict[tuple[int, int], list[dict[str, str]]]:
    rows: dict[tuple[int, int], list[dict[str, str]]] = {}
    for row in read_rows(path):
        key = (int(float(row["gps_week"])), round(float(row["gps_tow"])))
        rows.setdefault(key, []).append(row)
    return rows


def abs_diffs(
    keys: list[tuple[int, str, str]],
    cpp_rows: dict[tuple[int, str, str], dict[str, str]],
    taroz_rows: dict[tuple[int, str, str], dict[str, str]],
    cpp_column: str,
    taroz_column: str,
) -> list[float]:
    return [
        abs(float(cpp_rows[key][cpp_column]) - float(taroz_rows[key][taroz_column]))
        for key in keys
        if finite(cpp_rows[key][cpp_column]) and finite(taroz_rows[key][taroz_column])
    ]


def ecef_error_m(
    cpp_row: dict[str, float | int],
    taroz_row: dict[str, str],
    taroz_prefix: str,
) -> float:
    dx = float(cpp_row["x"]) - float(taroz_row[f"{taroz_prefix}_x_m"])
    dy = float(cpp_row["y"]) - float(taroz_row[f"{taroz_prefix}_y_m"])
    dz = float(cpp_row["z"]) - float(taroz_row[f"{taroz_prefix}_z_m"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def epoch_debug_ecef_error_m(
    cpp_row: dict[str, str],
    taroz_row: dict[str, str],
    taroz_prefix: str,
) -> float:
    dx = float(cpp_row["position_x_m"]) - float(taroz_row[f"{taroz_prefix}_x_m"])
    dy = float(cpp_row["position_y_m"]) - float(taroz_row[f"{taroz_prefix}_y_m"])
    dz = float(cpp_row["position_z_m"]) - float(taroz_row[f"{taroz_prefix}_z_m"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def pos_epoch_debug_ecef_error_m(
    pos_row: dict[str, float | int],
    epoch_row: dict[str, str],
) -> float:
    dx = float(pos_row["x"]) - float(epoch_row["position_x_m"])
    dy = float(pos_row["y"]) - float(epoch_row["position_y_m"])
    dz = float(pos_row["z"]) - float(epoch_row["position_z_m"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def epoch_debug_velocity_error_mps(
    cpp_row: dict[str, str],
    taroz_row: dict[str, str],
) -> float:
    dx = float(cpp_row["velocity_x_mps"]) - float(taroz_row["fgo_vx_mps"])
    dy = float(cpp_row["velocity_y_mps"]) - float(taroz_row["fgo_vy_mps"])
    dz = float(cpp_row["velocity_z_mps"]) - float(taroz_row["fgo_vz_mps"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def seed_position_error_m(
    cpp_row: dict[str, str],
    taroz_row: dict[str, str],
) -> float:
    dx = float(cpp_row["seed_position_x_m"]) - float(taroz_row["spp_x_m"])
    dy = float(cpp_row["seed_position_y_m"]) - float(taroz_row["spp_y_m"])
    dz = float(cpp_row["seed_position_z_m"]) - float(taroz_row["spp_z_m"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def delimited_count(value: str) -> int:
    return len([part for part in value.split(";") if part])


def lambda_diffs(
    keys: list[tuple[int, int, int, int, str, str]],
    cpp_rows: dict[tuple[int, int, int, int, str, str], dict[str, str]],
    taroz_rows: dict[tuple[int, int, int, int, str, str], dict[str, str]],
    column: str,
) -> list[float]:
    return [
        abs(float(cpp_rows[key][column]) - float(taroz_rows[key][column]))
        for key in keys
        if finite(cpp_rows[key][column]) and finite(taroz_rows[key][column])
    ]


def epoch_ratio_diffs(
    keys: list[tuple[int, int]],
    cpp_rows: dict[tuple[int, int], dict[str, str]],
    taroz_rows: dict[tuple[int, int], dict[str, str]],
) -> list[float]:
    return [
        abs(float(cpp_rows[key]["ratio"]) - float(taroz_rows[key]["ratio"]))
        for key in keys
        if finite(cpp_rows[key]["ratio"]) and finite(taroz_rows[key]["ratio"])
    ]


class TarozPosVelAmbPdcFactorParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [display_path(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest(
                "missing optional taroz position/velocity ambiguity PDC files: "
                + ", ".join(missing)
            )

    def test_counts_match_taroz_dump(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        cpp_epoch_path = CPP_DIR / "epoch_debug.csv"
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        taroz_factor_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(
            summary_path,
            cpp_epoch_path,
            cpp_factor_path,
            taroz_graph_path,
            taroz_epoch_path,
            taroz_factor_path,
        )

        summary = read_json(summary_path)
        taroz_graph = read_rows(taroz_graph_path)[0]
        cpp_epoch_rows = keyed_epoch_rows(cpp_epoch_path)
        taroz_epoch_rows = keyed_epoch_rows(taroz_epoch_path)
        cpp_rows = keyed_cpp_rows(cpp_factor_path)
        taroz_rows = keyed_taroz_rows(taroz_factor_path)
        carrier_keys = [
            key for key, row in cpp_rows.items()
            if finite(row["res_ldd"])
        ]

        self.assertEqual(summary["preset"], "taroz-amb-pdc")
        self.assertIsInstance(summary["debug_problem_only"], bool)
        self.assertEqual(summary["insert_fixed_interval_gaps"], True)
        self.assertEqual(summary["min_satellites_per_epoch"], 0)
        self.assertEqual(summary["min_output_dd_carrier_factors_per_epoch"], 6)
        self.assertEqual(summary["optimized_epochs"], int(float(taroz_graph["n"])))
        self.assertEqual(summary["use_single_difference_doppler_factors"], True)
        self.assertEqual(summary["use_single_difference_tdcp_factors"], True)
        self.assertIsInstance(summary["use_epoch_lambda_fixed_output"], bool)
        self.assertEqual(summary["double_difference_pseudorange_factors"], 15013)
        self.assertEqual(summary["double_difference_carrier_factors"], 13826)
        self.assertEqual(summary["single_difference_doppler_factors"], 18330)
        self.assertEqual(summary["single_difference_tdcp_factors"], 13509)
        self.assertEqual(summary["ambiguity_states"], 13826)
        self.assertEqual(len(cpp_rows), 15013)
        self.assertEqual(len(carrier_keys), 13826)
        self.assertEqual(set(cpp_rows), set(taroz_rows))
        self.assertEqual(set(cpp_epoch_rows), set(taroz_epoch_rows))
        for key, row in cpp_epoch_rows.items():
            self.assertEqual(
                int(row["ambiguity_candidates"]),
                int(float(taroz_epoch_rows[key]["candidate_count"])),
            )

    def test_solver_summary_matches_taroz_high_level_dump(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        self.require_dogfood_files(summary_path, taroz_graph_path)

        summary = read_json(summary_path)
        taroz_graph = read_rows(taroz_graph_path)[0]

        self.assertEqual(summary["preset"], "taroz-amb-pdc")
        self.assertEqual(summary["backend"], "eigen")
        self.assertEqual(
            int(summary["optimized_epochs"]),
            int(float(taroz_graph["n"])),
        )
        self.assertEqual(
            int(summary["ambiguity_states"]),
            int(float(taroz_graph["valid_ambiguity_states"])),
        )
        self.assertEqual(
            int(summary["fixed_solutions"]),
            int(float(taroz_graph["fixed_epochs"])),
        )
        self.assertEqual(int(summary["valid_solutions"]), 964)
        self.assertEqual(int(summary["float_solutions"]), 243)
        self.assertEqual(int(summary["graph_factors"]), 76357)
        self.assertEqual(int(summary["graph_values"]), 21813)
        self.assertLessEqual(
            abs(int(summary["iterations"]) - int(float(taroz_graph["iterations"]))),
            2,
        )
        self.assertTrue(math.isfinite(float(summary["initial_cost"])))
        self.assertTrue(math.isfinite(float(summary["final_cost"])))
        self.assertTrue(math.isfinite(float(taroz_graph["initial_cost"])))
        self.assertTrue(math.isfinite(float(taroz_graph["final_cost"])))
        self.assertLess(
            float(summary["final_cost"]),
            float(taroz_graph["initial_cost"]),
        )
        self.assertLessEqual(
            relative_error(
                float(summary["final_cost"]),
                float(taroz_graph["final_cost"]),
            ),
            0.30,
        )

    def test_cost_trace_matches_summary_and_taroz_graph_costs(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        self.require_dogfood_files(summary_path, CPP_OPT_COST_TRACE, taroz_graph_path)

        summary = read_json(summary_path)
        rows = read_rows(CPP_OPT_COST_TRACE)
        taroz_graph = read_rows(taroz_graph_path)[0]

        self.assertGreater(len(rows), 0)
        self.assertEqual(rows[0]["phase"], "float")
        self.assertEqual(int(rows[0]["local_iteration"]), 0)
        self.assertEqual(int(rows[0]["global_iteration"]), 0)
        self.assertTrue(all(row["phase"] in {"float", "fixed"} for row in rows))
        self.assertTrue(all(math.isfinite(float(row["cost"])) for row in rows))
        self.assertTrue(all(float(row["cost"]) >= 0.0 for row in rows))

        global_iterations = [int(row["global_iteration"]) for row in rows]
        self.assertEqual(global_iterations, sorted(global_iterations))
        self.assertEqual(len(global_iterations), len(set(global_iterations)))
        phase_count = len({row["phase"] for row in rows})
        self.assertLessEqual(len(rows), int(summary["iterations"]) + phase_count)

        self.assertEqual(float(rows[0]["cost"]), float(summary["initial_cost"]))
        self.assertEqual(float(rows[-1]["cost"]), float(summary["final_cost"]))
        self.assertLess(float(rows[-1]["cost"]), float(rows[0]["cost"]))
        self.assertLess(
            float(taroz_graph["final_cost"]),
            float(taroz_graph["initial_cost"]),
        )
        self.assertLessEqual(
            relative_error(float(rows[-1]["cost"]), float(taroz_graph["final_cost"])),
            0.30,
        )

    def test_matlab_optimizer_cost_trace_matches_taroz_graph_and_cpp_scale(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        self.require_dogfood_files(
            summary_path,
            CPP_OPT_COST_TRACE,
            taroz_graph_path,
            MATLAB_OPTIMIZER_COST_TRACE,
        )

        summary = read_json(summary_path)
        cpp_rows = read_rows(CPP_OPT_COST_TRACE)
        matlab_rows = read_rows(MATLAB_OPTIMIZER_COST_TRACE)
        taroz_graph = read_rows(taroz_graph_path)[0]

        self.assertGreater(len(matlab_rows), 1)
        self.assertTrue(all(row["phase"] == "gtsam" for row in matlab_rows))
        self.assertEqual(int(matlab_rows[0]["local_iteration"]), 0)
        self.assertEqual(int(matlab_rows[0]["global_iteration"]), 0)
        matlab_iterations = [int(row["global_iteration"]) for row in matlab_rows]
        self.assertEqual(matlab_iterations, sorted(matlab_iterations))
        self.assertEqual(len(matlab_iterations), len(set(matlab_iterations)))
        self.assertEqual(len(matlab_rows), int(float(taroz_graph["iterations"])) + 1)
        self.assertTrue(all(math.isfinite(float(row["cost"])) for row in matlab_rows))
        self.assertTrue(all(float(row["cost"]) >= 0.0 for row in matlab_rows))
        self.assertLess(float(matlab_rows[-1]["cost"]), float(matlab_rows[0]["cost"]))
        self.assertLess(float(cpp_rows[-1]["cost"]), float(cpp_rows[0]["cost"]))

        self.assertLessEqual(
            relative_error(
                float(matlab_rows[0]["cost"]),
                float(taroz_graph["initial_cost"]),
            ),
            1e-9,
        )
        self.assertLessEqual(
            relative_error(
                float(matlab_rows[-1]["cost"]),
                float(taroz_graph["final_cost"]),
            ),
            1e-9,
        )
        self.assertLessEqual(
            relative_error(float(cpp_rows[-1]["cost"]), float(matlab_rows[-1]["cost"])),
            0.30,
        )
        self.assertEqual(float(cpp_rows[-1]["cost"]), float(summary["final_cost"]))

    def test_seed_positions_match_taroz_spp_seed_dump(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        cpp_epoch_path = CPP_DIR / "epoch_debug.csv"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(summary_path, cpp_epoch_path, taroz_epoch_path)

        summary = read_json(summary_path)
        cpp_rows = keyed_epoch_rows(cpp_epoch_path)
        taroz_rows = keyed_epoch_rows(taroz_epoch_path)
        self.assertEqual(set(cpp_rows), set(taroz_rows))
        self.assertEqual(int(summary["seed_matched_epochs"]), len(cpp_rows))
        self.assertEqual(int(summary["seed_interpolated_epochs"]), 34)

        errors = [
            seed_position_error_m(cpp_rows[key], taroz_rows[key])
            for key in sorted(cpp_rows)
        ]
        self.assertEqual(len(errors), 1141)
        self.assertLessEqual(statistics.mean(errors), 1e-6)
        self.assertLessEqual(max(errors), 1e-6)

    def test_raw_dd_factors_match_taroz_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "factor_debug.csv"
        taroz_factor_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(cpp_factor_path, taroz_factor_path)

        cpp_rows = keyed_cpp_rows(cpp_factor_path)
        taroz_rows = keyed_taroz_rows(taroz_factor_path)
        self.assertEqual(set(cpp_rows), set(taroz_rows))
        keys = sorted(cpp_rows)
        carrier_keys = sorted(key for key in keys if finite(cpp_rows[key]["res_ldd"]))

        self.assertEqual(len(keys), 15013)
        self.assertEqual(len(carrier_keys), 13826)
        checks = [
            ("elevation_deg", "elevation_deg", 1e-3, 1e-3, 1e-3),
            ("sigma_pdd_m", "sigma_pdd_m", 1e-5, 2e-5, 2e-5),
            ("res_pdd", "res_pdd", 1e-4, 2e-4, 3e-4),
        ]
        for cpp_column, taroz_column, mean_limit, p95_limit, max_limit in checks:
            diffs = abs_diffs(keys, cpp_rows, taroz_rows, cpp_column, taroz_column)
            self.assertEqual(len(diffs), len(keys))
            self.assertLessEqual(statistics.mean(diffs), mean_limit)
            self.assertLessEqual(percentile(diffs, 0.95), p95_limit)
            self.assertLessEqual(max(diffs), max_limit)

        carrier_checks = [
            ("sigma_ldd_m", "sigma_ldd_m", 1e-7, 1e-7, 1e-7),
            ("res_ldd", "res_ldd", 1e-4, 2e-4, 3e-4),
            ("ambiguity_initial", "ambiguity_initial", 1e-8, 1e-8, 5e-8),
        ]
        for cpp_column, taroz_column, mean_limit, p95_limit, max_limit in carrier_checks:
            diffs = abs_diffs(carrier_keys, cpp_rows, taroz_rows, cpp_column, taroz_column)
            self.assertEqual(len(diffs), len(carrier_keys))
            self.assertLessEqual(statistics.mean(diffs), mean_limit)
            self.assertLessEqual(percentile(diffs, 0.95), p95_limit)
            self.assertLessEqual(max(diffs), max_limit)

    def test_optimized_solution_tracks_taroz_fixed_output_on_well_constrained_epochs(self) -> None:
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(CPP_OPT_POS, taroz_epoch_path)

        cpp_rows = read_pos_rows(CPP_OPT_POS)
        taroz_rows = {
            (int(float(row["gps_week"])), round(float(row["gps_tow"]))): row
            for row in read_rows(taroz_epoch_path)
        }
        self.assertEqual(set(cpp_rows), set(taroz_rows))

        well_constrained_keys = [
            key for key, row in taroz_rows.items()
            if int(float(row["candidate_count"])) >= 6
        ]
        self.assertEqual(len(well_constrained_keys), 964)
        self.assertEqual(
            Counter(int(row["status"]) for row in cpp_rows.values()),
            Counter({4: 721, 3: 243, 0: 177}),
        )
        valid_cpp_keys = {
            key for key, row in cpp_rows.items()
            if int(row["status"]) != 0 and int(row["satellites"]) >= 4
        }
        self.assertEqual(valid_cpp_keys, set(well_constrained_keys))
        cpp_fixed_keys = {
            key for key, row in cpp_rows.items()
            if int(row["status"]) == 4
        }
        taroz_fixed_valid_keys = {
            key for key, row in taroz_rows.items()
            if int(float(row["idxfix"])) == 1 and key in valid_cpp_keys
        }
        self.assertEqual(len(cpp_fixed_keys), 721)
        self.assertEqual(len(valid_cpp_keys - cpp_fixed_keys), 243)
        self.assertEqual(cpp_fixed_keys, taroz_fixed_valid_keys)
        for key, row in cpp_rows.items():
            self.assertEqual(
                int(row["fixed_ambiguities"]),
                int(float(taroz_rows[key]["fixed_ambiguity_count"])),
            )
            if int(row["status"]) == 0:
                self.assertEqual(float(row["ratio"]), 0.0)
                self.assertEqual(int(row["fixed_ambiguities"]), 0)
            else:
                taroz_fixed = int(float(taroz_rows[key]["idxfix"])) == 1
                self.assertEqual(int(row["status"]) == 4, taroz_fixed)

        errors = [
            ecef_error_m(cpp_rows[key], taroz_rows[key], "fixed")
            for key in well_constrained_keys
        ]
        self.assertLessEqual(statistics.mean(errors), 0.001)
        self.assertLessEqual(statistics.median(errors), 0.0002)
        self.assertLessEqual(percentile(errors, 0.95), 0.0012)
        self.assertLessEqual(max(errors), 0.012)

        ratio_diffs = [
            abs(float(cpp_rows[key]["ratio"]) - float(taroz_rows[key]["ratio"]))
            for key in well_constrained_keys
            if finite(taroz_rows[key]["ratio"])
        ]
        self.assertEqual(len(ratio_diffs), len(well_constrained_keys))
        self.assertLessEqual(statistics.mean(ratio_diffs), 0.007)
        self.assertLessEqual(statistics.median(ratio_diffs), 0.004)
        self.assertLessEqual(percentile(ratio_diffs, 0.95), 0.025)
        self.assertLessEqual(max(ratio_diffs), 0.08)

    def test_final_pos_file_matches_epoch_debug_output_contract(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        self.require_dogfood_files(CPP_OPT_POS, CPP_OPT_EPOCH_DEBUG, summary_path)

        summary = read_json(summary_path)
        pos_rows = read_pos_rows(CPP_OPT_POS)
        epoch_rows = keyed_epoch_rows(CPP_OPT_EPOCH_DEBUG)
        self.assertEqual(set(pos_rows), set(epoch_rows))
        self.assertEqual(len(pos_rows), 1141)

        expected_iterations = int(summary["iterations"])
        self.assertEqual(
            Counter(int(row["iterations"]) for row in pos_rows.values()),
            Counter({expected_iterations: len(pos_rows)}),
        )
        self.assertEqual(
            Counter(int(row["status"]) for row in pos_rows.values()),
            Counter(int(row["status"]) for row in epoch_rows.values()),
        )

        position_errors: list[float] = []
        ratio_diffs: list[float] = []
        for key, pos_row in pos_rows.items():
            epoch_row = epoch_rows[key]
            self.assertEqual(int(pos_row["status"]), int(epoch_row["status"]))
            self.assertEqual(
                int(pos_row["satellites"]),
                delimited_count(epoch_row["dd_satellites"]),
            )
            self.assertEqual(
                int(pos_row["fixed_ambiguities"]),
                int(epoch_row["num_fixed_ambiguities"]),
            )
            if int(pos_row["status"]) == 0:
                self.assertEqual(float(pos_row["ratio"]), 0.0)
                self.assertEqual(int(pos_row["fixed_ambiguities"]), 0)

            position_errors.append(pos_epoch_debug_ecef_error_m(pos_row, epoch_row))
            ratio_diffs.append(
                abs(float(pos_row["ratio"]) - float(epoch_row["ratio"]))
            )

        self.assertLessEqual(max(position_errors), 2e-6)
        self.assertLessEqual(max(ratio_diffs), 5e-7)

    def test_optimized_epoch_debug_tracks_taroz_fixed_output(self) -> None:
        taroz_epoch_path = MATLAB_DIR / "per_epoch_state.csv"
        self.require_dogfood_files(CPP_OPT_EPOCH_DEBUG, taroz_epoch_path)

        cpp_rows = keyed_epoch_rows(CPP_OPT_EPOCH_DEBUG)
        taroz_rows = keyed_epoch_rows(taroz_epoch_path)
        self.assertEqual(set(cpp_rows), set(taroz_rows))

        well_constrained_keys = [
            key for key, row in taroz_rows.items()
            if int(float(row["candidate_count"])) >= 6
        ]
        self.assertEqual(len(well_constrained_keys), 964)
        self.assertEqual(
            Counter(int(row["status"]) for row in cpp_rows.values()),
            Counter({4: 721, 3: 243, 0: 177}),
        )
        valid_cpp_keys = {
            key for key, row in cpp_rows.items()
            if (
                int(row["status"]) != 0 and
                int(row["ambiguity_candidates"]) >= 6
            )
        }
        self.assertEqual(valid_cpp_keys, set(well_constrained_keys))
        cpp_fixed_keys = {
            key for key, row in cpp_rows.items()
            if int(row["status"]) == 4
        }
        taroz_fixed_valid_keys = {
            key for key, row in taroz_rows.items()
            if int(float(row["idxfix"])) == 1 and key in valid_cpp_keys
        }
        self.assertEqual(len(cpp_fixed_keys), 721)
        self.assertEqual(len(valid_cpp_keys - cpp_fixed_keys), 243)
        self.assertEqual(cpp_fixed_keys, taroz_fixed_valid_keys)
        for key, row in cpp_rows.items():
            self.assertEqual(
                int(row["ambiguity_candidates"]),
                int(float(taroz_rows[key]["candidate_count"])),
            )
            self.assertEqual(
                int(row["num_fixed_ambiguities"]),
                int(float(taroz_rows[key]["fixed_ambiguity_count"])),
            )

        self.assertEqual(
            sum(int(row["num_fixed_ambiguities"]) for row in cpp_rows.values()),
            10737,
        )

        ratio_diffs = epoch_ratio_diffs(well_constrained_keys, cpp_rows, taroz_rows)
        self.assertEqual(len(ratio_diffs), len(well_constrained_keys))
        self.assertLessEqual(statistics.mean(ratio_diffs), 0.007)
        self.assertLessEqual(statistics.median(ratio_diffs), 0.004)
        self.assertLessEqual(percentile(ratio_diffs, 0.95), 0.025)
        self.assertLessEqual(max(ratio_diffs), 0.08)
        for key in well_constrained_keys:
            cpp_fixed = int(cpp_rows[key]["status"]) == 4
            taroz_fixed = int(float(taroz_rows[key]["idxfix"])) == 1
            self.assertEqual(cpp_fixed, taroz_fixed)

        position_errors = [
            epoch_debug_ecef_error_m(cpp_rows[key], taroz_rows[key], "fixed")
            for key in well_constrained_keys
        ]
        self.assertLessEqual(statistics.mean(position_errors), 0.001)
        self.assertLessEqual(statistics.median(position_errors), 0.0002)
        self.assertLessEqual(percentile(position_errors, 0.95), 0.0012)
        self.assertLessEqual(max(position_errors), 0.012)

        velocity_errors = [
            epoch_debug_velocity_error_mps(cpp_rows[key], taroz_rows[key])
            for key in well_constrained_keys
            if (
                finite(cpp_rows[key]["velocity_x_mps"]) and
                finite(cpp_rows[key]["velocity_y_mps"]) and
                finite(cpp_rows[key]["velocity_z_mps"])
            )
        ]
        self.assertGreaterEqual(len(velocity_errors), len(well_constrained_keys) - 2)
        self.assertLessEqual(statistics.mean(velocity_errors), 0.001)
        self.assertLessEqual(statistics.median(velocity_errors), 0.0003)
        self.assertLessEqual(percentile(velocity_errors, 0.95), 0.003)
        self.assertLessEqual(max(velocity_errors), 0.012)

    def test_lambda_debug_tracks_taroz_internal_ambiguity_state(self) -> None:
        taroz_lambda_path = MATLAB_DIR / "per_lambda_detail.csv"
        self.require_dogfood_files(CPP_OPT_LAMBDA_DEBUG, taroz_lambda_path)

        cpp_rows = keyed_lambda_rows(CPP_OPT_LAMBDA_DEBUG)
        taroz_rows = keyed_lambda_rows(taroz_lambda_path)
        self.assertEqual(len(cpp_rows), 208172)
        self.assertEqual(set(cpp_rows), set(taroz_rows))
        keys = sorted(cpp_rows)

        for key in keys:
            self.assertEqual(
                int(cpp_rows[key]["candidate_count"]),
                int(float(taroz_rows[key]["candidate_count"])),
            )

        diagonal_keys = [
            key for key in keys
            if key[2] == 0 and key[3] == 0
        ]
        self.assertEqual(len(diagonal_keys), 964)
        self.assertTrue(all(int(cpp_rows[key]["solved"]) == 1 for key in diagonal_keys))
        cpp_fixed_epochs = sum(int(cpp_rows[key]["fixed_epoch"]) for key in diagonal_keys)
        taroz_fixed_epochs = sum(
            int(float(taroz_rows[key]["fixed_epoch"])) for key in diagonal_keys
        )
        self.assertEqual(cpp_fixed_epochs, 721)
        self.assertEqual(cpp_fixed_epochs, taroz_fixed_epochs)
        for key in diagonal_keys:
            self.assertEqual(
                int(cpp_rows[key]["fixed_epoch"]),
                int(float(taroz_rows[key]["fixed_epoch"])),
            )

        checks = [
            ("ambiguity_float", 0.0015, 0.0012, 0.003, 0.03),
            ("fixed_ambiguity", 1e-8, 1e-8, 1e-8, 1e-8),
            ("covariance", 3e-6, 2e-6, 8e-6, 5e-4),
            ("position_covariance_x", 1e-6, 6e-7, 2e-6, 1e-4),
            ("position_covariance_y", 1e-6, 6e-7, 2e-6, 1e-4),
            ("position_covariance_z", 1e-6, 6e-7, 2e-6, 1e-4),
            ("ratio", 0.007, 0.004, 0.025, 0.08),
        ]
        for column, mean_limit, median_limit, p95_limit, max_limit in checks:
            diffs = lambda_diffs(keys, cpp_rows, taroz_rows, column)
            self.assertEqual(len(diffs), len(keys))
            self.assertLessEqual(statistics.mean(diffs), mean_limit)
            self.assertLessEqual(statistics.median(diffs), median_limit)
            self.assertLessEqual(percentile(diffs, 0.95), p95_limit)
            self.assertLessEqual(max(diffs), max_limit)

    def test_lambda_debug_matrix_contract_matches_epoch_debug(self) -> None:
        summary_path = CPP_DIR / "summary.json"
        self.require_dogfood_files(
            summary_path,
            CPP_OPT_EPOCH_DEBUG,
            CPP_OPT_LAMBDA_DEBUG,
        )

        summary = read_json(summary_path)
        epoch_rows = keyed_epoch_rows(CPP_OPT_EPOCH_DEBUG)
        lambda_by_epoch = grouped_lambda_rows(CPP_OPT_LAMBDA_DEBUG)
        min_candidates = int(summary["min_output_dd_carrier_factors_per_epoch"])
        attempted_epochs = {
            key: row for key, row in epoch_rows.items()
            if int(row["ambiguity_candidates"]) >= min_candidates
        }

        self.assertEqual(set(lambda_by_epoch), set(attempted_epochs))
        self.assertEqual(
            len(lambda_by_epoch),
            int(summary["lambda_ambiguity_attempts"]),
        )

        lambda_rows = 0
        lambda_candidates = 0
        fixed_candidates = 0
        fixed_epochs = 0
        covariance_symmetry_diffs: list[float] = []
        for key, rows in lambda_by_epoch.items():
            epoch_row = attempted_epochs[key]
            candidate_count = int(epoch_row["ambiguity_candidates"])
            lambda_candidates += candidate_count
            self.assertEqual(len(rows), candidate_count * candidate_count)

            lookup: dict[tuple[int, int], dict[str, str]] = {}
            row_satellites: dict[int, str] = {}
            col_satellites: dict[int, str] = {}
            for row in rows:
                lambda_rows += 1
                row_index = int(row["row"])
                col_index = int(row["col"])
                self.assertGreaterEqual(row_index, 0)
                self.assertLess(row_index, candidate_count)
                self.assertGreaterEqual(col_index, 0)
                self.assertLess(col_index, candidate_count)
                self.assertNotIn((row_index, col_index), lookup)
                lookup[(row_index, col_index)] = row

                self.assertEqual(int(row["solved"]), 1)
                self.assertEqual(int(row["candidate_count"]), candidate_count)
                self.assertEqual(int(row["local_index"]), row_index)
                self.assertEqual(int(row["other_local_index"]), col_index)
                self.assertAlmostEqual(
                    float(row["ratio"]),
                    float(epoch_row["ratio"]),
                    places=6,
                )

                previous_row_satellite = row_satellites.setdefault(
                    row_index,
                    row["satellite"],
                )
                previous_col_satellite = col_satellites.setdefault(
                    col_index,
                    row["other_satellite"],
                )
                self.assertEqual(previous_row_satellite, row["satellite"])
                self.assertEqual(previous_col_satellite, row["other_satellite"])

            self.assertEqual(
                set(lookup),
                {
                    (row_index, col_index)
                    for row_index in range(candidate_count)
                    for col_index in range(candidate_count)
                },
            )

            fixed_epoch = int(lookup[(0, 0)]["fixed_epoch"]) == 1
            self.assertEqual(fixed_epoch, int(epoch_row["status"]) == 4)
            if fixed_epoch:
                fixed_epochs += 1
                fixed_candidates += candidate_count
            for index in range(candidate_count):
                self.assertEqual(row_satellites[index], col_satellites[index])
                diagonal_row = lookup[(index, index)]
                self.assertEqual(
                    diagonal_row["satellite"],
                    diagonal_row["other_satellite"],
                )
                self.assertGreater(float(diagonal_row["covariance"]), 0.0)

            for row in rows:
                self.assertEqual(int(row["fixed_epoch"]) == 1, fixed_epoch)

            for row_index in range(candidate_count):
                for col_index in range(candidate_count):
                    covariance_symmetry_diffs.append(
                        abs(
                            float(lookup[(row_index, col_index)]["covariance"])
                            - float(lookup[(col_index, row_index)]["covariance"])
                        )
                    )

        self.assertEqual(lambda_rows, len(read_rows(CPP_OPT_LAMBDA_DEBUG)))
        self.assertEqual(lambda_candidates, int(summary["lambda_ambiguity_candidates"]))
        self.assertEqual(
            fixed_candidates,
            int(summary["lambda_ambiguity_used_candidates"]),
        )
        self.assertEqual(fixed_epochs, int(summary["fixed_solutions"]))
        self.assertLessEqual(max(covariance_symmetry_diffs), 2e-12)

    def test_raw_sd_doppler_tdcp_match_taroz_dump(self) -> None:
        cpp_sd_path = CPP_DIR / "sd_factor_debug.csv"
        taroz_factor_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(cpp_sd_path, taroz_factor_path)

        cpp_doppler_rows = keyed_valid_rows(cpp_sd_path, "valid_doppler_sd")
        cpp_tdcp_rows = keyed_valid_rows(cpp_sd_path, "valid_tdcp_sd")
        taroz_doppler_rows = keyed_valid_rows(taroz_factor_path, "valid_doppler_sd")
        taroz_tdcp_rows = keyed_valid_rows(taroz_factor_path, "valid_tdcp_sd")

        self.assertEqual(len(cpp_doppler_rows), 18330)
        self.assertEqual(len(cpp_tdcp_rows), 13509)
        self.assertEqual(set(cpp_doppler_rows), set(taroz_doppler_rows))
        self.assertEqual(set(cpp_tdcp_rows), set(taroz_tdcp_rows))

        doppler_keys = sorted(cpp_doppler_rows)
        tdcp_keys = sorted(cpp_tdcp_rows)
        doppler_checks = [
            ("sigma_dsd_mps", "sigma_dsd_mps", 1e-6, 3e-6, 4e-6),
            ("res_dsd_mps", "res_dsd_mps", 1e-4, 2e-4, 5e-4),
            ("los_x", "los_x", 4e-6, 8e-6, 8e-6),
            ("los_y", "los_y", 4e-6, 8e-6, 8e-6),
            ("los_z", "los_z", 5e-7, 1e-6, 1e-6),
        ]
        for cpp_column, taroz_column, mean_limit, p95_limit, max_limit in doppler_checks:
            diffs = abs_diffs(
                doppler_keys,
                cpp_doppler_rows,
                taroz_doppler_rows,
                cpp_column,
                taroz_column,
            )
            self.assertEqual(len(diffs), len(doppler_keys))
            self.assertLessEqual(statistics.mean(diffs), mean_limit)
            self.assertLessEqual(percentile(diffs, 0.95), p95_limit)
            self.assertLessEqual(max(diffs), max_limit)

        tdcp_checks = [
            ("sigma_tdcp_m", "sigma_ldd_m", 1e-7, 1e-7, 1e-7),
            ("tdcp_sd_m", "tdcp_sd_m", 1e-4, 2e-4, 3e-1),
        ]
        for cpp_column, taroz_column, mean_limit, p95_limit, max_limit in tdcp_checks:
            diffs = abs_diffs(
                tdcp_keys,
                cpp_tdcp_rows,
                taroz_tdcp_rows,
                cpp_column,
                taroz_column,
            )
            self.assertEqual(len(diffs), len(tdcp_keys))
            self.assertLessEqual(statistics.mean(diffs), mean_limit)
            self.assertLessEqual(percentile(diffs, 0.95), p95_limit)
            self.assertLessEqual(max(diffs), max_limit)


if __name__ == "__main__":
    unittest.main()
