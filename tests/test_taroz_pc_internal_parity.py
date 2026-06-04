#!/usr/bin/env python3
"""Optional parity checks against taroz MATLAB PPC dogfood dumps."""

from __future__ import annotations

import csv
from collections import Counter
import json
import math
import os
from pathlib import Path
import unittest
from typing import Any


ROOT_DIR = Path(__file__).resolve().parents[1]


def configured_path(env_name: str, default_relative_path: str) -> Path:
    configured = os.environ.get(env_name)
    if configured:
        path = Path(configured)
        return path if path.is_absolute() else ROOT_DIR / path
    return ROOT_DIR / default_relative_path


CPP_DIR = configured_path(
    "GNSSPP_TAROZ_PC_CPP_DIR",
    "output/dogfood/taroz_pc_full_gtsam_fixed_taroz_nocorr",
)
CPP_LAMBDA_DIR = configured_path(
    "GNSSPP_TAROZ_PC_CPP_LAMBDA_DIR",
    "output/dogfood/taroz_pc_lambda_debug",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_PC_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pc_debug",
)


def finite_float(value: str | None) -> float | None:
    if value is None or value == "":
        return None
    parsed = float(value)
    return parsed if math.isfinite(parsed) else None


def percentile(values: list[float], percent: float) -> float:
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    position = (len(ordered) - 1) * percent / 100.0
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


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


def read_pos_details_by_tow(path: Path) -> dict[int, dict[str, float | int]]:
    rows: dict[int, dict[str, float | int]] = {}
    with path.open(encoding="ascii", errors="ignore") as handle:
        for line in handle:
            if not line.strip() or line.startswith("%"):
                continue
            parts = line.split()
            rows[round(float(parts[1]))] = {
                "gps_week": int(float(parts[0])),
                "gps_tow": float(parts[1]),
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


def delimited_count(value: str) -> int:
    return len([part for part in value.split(";") if part])


def pos_epoch_debug_ecef_error_m(
    pos_row: dict[str, float | int],
    epoch_row: dict[str, str],
) -> float:
    dx = float(pos_row["x"]) - float(epoch_row["position_x_m"])
    dy = float(pos_row["y"]) - float(epoch_row["position_y_m"])
    dz = float(pos_row["z"]) - float(epoch_row["position_z_m"])
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class TarozPcInternalParityTest(unittest.TestCase):
    def require_dogfood_files(self, *paths: Path) -> None:
        missing = [str(path.relative_to(ROOT_DIR)) for path in paths if not path.exists()]
        if missing:
            self.skipTest("missing optional taroz dogfood files: " + ", ".join(missing))

    def test_epoch_fix_ratio_and_position_match_taroz_dump(self) -> None:
        cpp_epoch_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed_epoch_debug.csv"
        cpp_pos_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed.pos"
        taroz_epoch_path = MATLAB_DIR / "per_epoch_fix_detail.csv"
        self.require_dogfood_files(cpp_epoch_path, cpp_pos_path, taroz_epoch_path)

        cpp_epochs = read_csv_by_tow(cpp_epoch_path)
        cpp_positions = read_pos_by_tow(cpp_pos_path)
        taroz_epochs = read_csv_by_tow(taroz_epoch_path)
        common_tows = sorted(set(cpp_epochs) & set(cpp_positions) & set(taroz_epochs))
        self.assertGreaterEqual(len(common_tows), 1000)

        candidate_mismatches: list[int] = []
        fixed_mismatches: list[int] = []
        cxx_only_fixed: list[int] = []
        ratio_diffs: list[float] = []
        same_status_position_diffs: list[float] = []
        allowed_taroz_only_fixed = {554262, 554360}
        for tow in common_tows:
            cpp_epoch = cpp_epochs[tow]
            taroz_epoch = taroz_epochs[tow]
            if int(cpp_epoch["ambiguity_candidates"]) != int(taroz_epoch["candidate_count"]):
                candidate_mismatches.append(tow)

            cpp_fixed = int(cpp_epoch["status"]) == 4
            taroz_fixed = int(float(taroz_epoch["idxfix"])) == 1
            if cpp_fixed != taroz_fixed:
                fixed_mismatches.append(tow)
                if cpp_fixed:
                    cxx_only_fixed.append(tow)

            ratio_diffs.append(abs(float(cpp_epoch["ratio"]) - float(taroz_epoch["ratio"])))

            if cpp_fixed == taroz_fixed:
                x, y, z, _status = cpp_positions[tow]
                tx = float(taroz_epoch["fixed_x_m"])
                ty = float(taroz_epoch["fixed_y_m"])
                tz = float(taroz_epoch["fixed_z_m"])
                same_status_position_diffs.append(math.dist((x, y, z), (tx, ty, tz)))

        self.assertEqual(candidate_mismatches, [])
        self.assertLessEqual(set(fixed_mismatches), allowed_taroz_only_fixed)
        self.assertEqual(cxx_only_fixed, [])
        self.assertLessEqual(sum(ratio_diffs) / len(ratio_diffs), 0.004)
        self.assertLessEqual(max(ratio_diffs), 0.07)
        self.assertLessEqual(max(same_status_position_diffs), 0.01)

    def test_final_pos_file_matches_epoch_debug_output_contract(self) -> None:
        cpp_epoch_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed_epoch_debug.csv"
        cpp_pos_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed.pos"
        cpp_summary_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed_summary.json"
        self.require_dogfood_files(cpp_epoch_path, cpp_pos_path, cpp_summary_path)

        cpp_epochs = read_csv_by_tow(cpp_epoch_path)
        cpp_positions = read_pos_details_by_tow(cpp_pos_path)
        with cpp_summary_path.open() as handle:
            summary: dict[str, Any] = json.load(handle)

        self.assertEqual(set(cpp_positions), set(cpp_epochs))
        self.assertEqual(len(cpp_positions), int(summary["optimized_epochs"]))
        self.assertEqual(
            Counter(int(row["status"]) for row in cpp_positions.values()),
            Counter(int(row["status"]) for row in cpp_epochs.values()),
        )
        self.assertEqual(
            Counter(int(row["iterations"]) for row in cpp_positions.values()),
            Counter({int(summary["iterations"]): len(cpp_positions)}),
        )

        position_errors: list[float] = []
        ratio_diffs: list[float] = []
        for tow, pos_row in cpp_positions.items():
            epoch_row = cpp_epochs[tow]
            self.assertEqual(int(pos_row["status"]), int(epoch_row["status"]))
            self.assertGreaterEqual(
                int(pos_row["satellites"]),
                delimited_count(epoch_row["dd_satellites"]),
            )
            self.assertEqual(
                int(pos_row["fixed_ambiguities"]),
                int(epoch_row["num_fixed_ambiguities"]),
            )
            if int(pos_row["status"]) == 3:
                self.assertEqual(int(pos_row["fixed_ambiguities"]), 0)
            else:
                self.assertEqual(int(pos_row["status"]), 4)
                self.assertGreater(int(pos_row["fixed_ambiguities"]), 0)

            if all(
                column in epoch_row
                for column in ("position_x_m", "position_y_m", "position_z_m")
            ):
                position_errors.append(
                    pos_epoch_debug_ecef_error_m(pos_row, epoch_row)
                )
            ratio_diffs.append(
                abs(float(pos_row["ratio"]) - float(epoch_row["ratio"]))
            )

        if position_errors:
            self.assertLessEqual(max(position_errors), 2e-6)
        self.assertLessEqual(max(ratio_diffs), 5e-7)

    def test_factor_residuals_match_taroz_per_satellite_dump(self) -> None:
        cpp_factor_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed_factor_debug.csv"
        taroz_sat_path = MATLAB_DIR / "per_sat_detail.csv"
        self.require_dogfood_files(cpp_factor_path, taroz_sat_path)

        def key(row: dict[str, str]) -> tuple[int, str, str]:
            return (
                round(float(row["gps_tow"])),
                row["satellite"],
                row["reference"],
            )

        with cpp_factor_path.open(newline="") as handle:
            cpp_rows = {key(row): row for row in csv.DictReader(handle)}
        with taroz_sat_path.open(newline="") as handle:
            taroz_rows = {key(row): row for row in csv.DictReader(handle)}

        common_keys = sorted(set(cpp_rows) & set(taroz_rows))
        self.assertGreaterEqual(len(common_keys), 14000)
        self.assertEqual(sorted(set(cpp_rows) - set(taroz_rows)), [])

        def abs_diffs(column: str) -> list[float]:
            diffs: list[float] = []
            for row_key in common_keys:
                cpp_value = finite_float(cpp_rows[row_key].get(column))
                taroz_value = finite_float(taroz_rows[row_key].get(column))
                if cpp_value is None or taroz_value is None:
                    continue
                diffs.append(abs(cpp_value - taroz_value))
            return diffs

        res_pdd = abs_diffs("res_pdd")
        res_ldd = abs_diffs("res_ldd")
        elevation = abs_diffs("elevation_deg")
        sigma_pdd = abs_diffs("sigma_pdd_m")
        sigma_ldd = abs_diffs("sigma_ldd_m")
        ambiguity_initial = abs_diffs("ambiguity_initial")
        ambiguity_estimate = abs_diffs("ambiguity_estimate")
        self.assertGreaterEqual(len(res_ldd), 13000)
        self.assertLessEqual(max(elevation), 1e-3)
        self.assertLessEqual(max(sigma_pdd), 2e-5)
        self.assertLessEqual(max(sigma_ldd), 1e-7)
        self.assertLessEqual(max(res_pdd), 3e-4)
        self.assertLessEqual(max(res_ldd), 3e-4)
        self.assertLessEqual(max(ambiguity_initial), 1e-6)
        self.assertLessEqual(percentile(ambiguity_estimate, 95.0), 5e-4)
        self.assertLessEqual(max(ambiguity_estimate), 0.03)

    def test_graph_diagnostics_stay_close_to_taroz_dump(self) -> None:
        cpp_summary_path = CPP_DIR / "fgo_taroz_pc_gtsam_fixed_summary.json"
        taroz_graph_path = MATLAB_DIR / "graph_detail.csv"
        self.require_dogfood_files(cpp_summary_path, taroz_graph_path)

        with cpp_summary_path.open() as handle:
            cpp = json.load(handle)
        with taroz_graph_path.open(newline="") as handle:
            taroz = next(csv.DictReader(handle))

        self.assertFalse(cpp["use_spp_seed"])

        taroz_graph_factors = int(taroz["graph_factors"])
        taroz_graph_values = int(taroz["graph_values"])
        taroz_initial_cost = float(taroz["initial_cost"])
        taroz_final_cost = float(taroz["final_cost"])

        self.assertEqual(int(cpp["iterations"]), int(taroz["iterations"]))
        self.assertLessEqual(int(cpp["optimized_epochs"]), int(taroz["n"]))
        self.assertLessEqual(
            abs(int(cpp["optimized_epochs"]) - int(taroz["valid_solution_epochs"])),
            30,
        )
        self.assertLessEqual(
            abs(int(cpp["ambiguity_states"]) - int(taroz["valid_ambiguity_states"])),
            20,
        )
        self.assertLessEqual(
            abs(int(cpp["graph_factors"]) - taroz_graph_factors) / taroz_graph_factors,
            0.01,
        )
        self.assertLessEqual(
            abs(int(cpp["graph_values"]) - taroz_graph_values) / taroz_graph_values,
            0.06,
        )
        self.assertLessEqual(
            abs(float(cpp["initial_cost"]) - taroz_initial_cost) / taroz_initial_cost,
            0.002,
        )
        self.assertLessEqual(
            abs(float(cpp["final_cost"]) - taroz_final_cost) / taroz_final_cost,
            0.01,
        )

    def test_lambda_inputs_match_taroz_internal_dump(self) -> None:
        cpp_lambda_path = (
            CPP_LAMBDA_DIR / "fgo_taroz_pc_gtsam_fixed_lambda_debug.csv"
        )
        taroz_lambda_path = MATLAB_DIR / "per_lambda_detail.csv"
        self.require_dogfood_files(cpp_lambda_path, taroz_lambda_path)

        def key(row: dict[str, str]) -> tuple[int, int, int, str, str]:
            return (
                round(float(row["gps_tow"])),
                int(row["row"]),
                int(row["col"]),
                row["satellite"],
                row["other_satellite"],
            )

        with cpp_lambda_path.open(newline="") as handle:
            cpp_rows = {key(row): row for row in csv.DictReader(handle)}
        with taroz_lambda_path.open(newline="") as handle:
            taroz_rows = {key(row): row for row in csv.DictReader(handle)}

        common_keys = sorted(set(cpp_rows) & set(taroz_rows))
        self.assertGreaterEqual(len(common_keys), 200000)
        self.assertEqual(sorted(set(cpp_rows) - set(taroz_rows)), [])
        self.assertEqual(sorted(set(taroz_rows) - set(cpp_rows)), [])

        def abs_diffs(column: str) -> list[float]:
            return [
                abs(float(cpp_rows[row_key][column]) - float(taroz_rows[row_key][column]))
                for row_key in common_keys
            ]

        ambiguity_float = abs_diffs("ambiguity_float")
        covariance = abs_diffs("covariance")
        position_cov_x = abs_diffs("position_covariance_x")
        position_cov_y = abs_diffs("position_covariance_y")
        position_cov_z = abs_diffs("position_covariance_z")

        ratio_diffs: list[float] = []
        used_fixed_ambiguity_diffs: list[float] = []
        unused_fixed_ambiguity_mismatch_rows = 0
        seen_tows: set[int] = set()
        for row_key in common_keys:
            fixed_ambiguity_diff = abs(
                float(cpp_rows[row_key]["fixed_ambiguity"]) -
                float(taroz_rows[row_key]["fixed_ambiguity"])
            )
            if (int(cpp_rows[row_key]["fixed_epoch"]) == 1 or
                    int(taroz_rows[row_key]["fixed_epoch"]) == 1):
                used_fixed_ambiguity_diffs.append(fixed_ambiguity_diff)
            elif fixed_ambiguity_diff > 1e-6:
                unused_fixed_ambiguity_mismatch_rows += 1

            tow = row_key[0]
            if tow in seen_tows:
                continue
            seen_tows.add(tow)
            ratio_diffs.append(
                abs(float(cpp_rows[row_key]["ratio"]) -
                    float(taroz_rows[row_key]["ratio"]))
            )

        self.assertGreaterEqual(len(seen_tows), 900)
        self.assertLessEqual(percentile(ambiguity_float, 95.0), 4e-4)
        self.assertLessEqual(max(ambiguity_float), 0.03)
        self.assertLessEqual(max(used_fixed_ambiguity_diffs), 1e-6)
        self.assertLessEqual(unused_fixed_ambiguity_mismatch_rows, 300)
        self.assertLessEqual(percentile(covariance, 95.0), 1e-4)
        self.assertLessEqual(max(covariance), 0.02)
        self.assertLessEqual(percentile(position_cov_x, 95.0), 3e-5)
        self.assertLessEqual(max(position_cov_x), 0.003)
        self.assertLessEqual(percentile(position_cov_y, 95.0), 4e-5)
        self.assertLessEqual(max(position_cov_y), 0.002)
        self.assertLessEqual(percentile(position_cov_z, 95.0), 2e-5)
        self.assertLessEqual(max(position_cov_z), 0.002)
        self.assertLessEqual(sum(ratio_diffs) / len(ratio_diffs), 0.005)
        self.assertLessEqual(max(ratio_diffs), 0.07)


if __name__ == "__main__":
    unittest.main()
