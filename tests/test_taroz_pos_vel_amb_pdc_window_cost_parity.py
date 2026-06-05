#!/usr/bin/env python3
"""Optional windowed cost parity for taroz position/velocity ambiguity PDC."""

from __future__ import annotations

import csv
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
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_DIR",
    "output/dogfood/taroz_pos_vel_amb_pdc_first_120_profile_current",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_first_120_debug",
)
CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_SUMMARY",
    "output/dogfood/taroz_pos_vel_amb_pdc_first_120_profile_current/summary.json",
)
CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_COST_TRACE",
    "output/dogfood/taroz_pos_vel_amb_pdc_first_120_profile_current/cost_trace.csv",
)
CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_CPP_EPOCH_DEBUG",
    "output/dogfood/taroz_pos_vel_amb_pdc_first_120_profile_current/epoch_debug.csv",
)
MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_first_120_debug/graph_detail.csv",
)
MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_first_120_debug/optimizer_cost_trace.csv",
)
MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_WINDOW_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_first_120_debug/per_epoch_state.csv",
)


def display_path(path: Path) -> str:
    try:
        return str(path.relative_to(ROOT_DIR))
    except ValueError:
        return str(path)


def read_rows(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def read_json(path: Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def relative_error(actual: float, expected: float) -> float:
    return abs(actual - expected) / max(1.0, abs(expected))


def finite_float(value: object) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"not finite: {value!r}")
    return result


def epoch_keys(rows: list[dict[str, str]]) -> list[tuple[int, int]]:
    return [
        (int(float(row["gps_week"])), round(float(row["gps_tow"])))
        for row in rows
    ]


def keyed_rows(rows: list[dict[str, str]]) -> dict[tuple[int, int], dict[str, str]]:
    return dict(zip(epoch_keys(rows), rows))


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("cannot compute percentile of an empty list")
    sorted_values = sorted(values)
    index = math.ceil(fraction * len(sorted_values)) - 1
    return sorted_values[max(0, min(index, len(sorted_values) - 1))]


def ecef_error_m(
    cpp_row: dict[str, str],
    matlab_row: dict[str, str],
    matlab_prefix: str,
) -> float:
    dx = finite_float(cpp_row["position_x_m"]) - finite_float(
        matlab_row[f"{matlab_prefix}_x_m"]
    )
    dy = finite_float(cpp_row["position_y_m"]) - finite_float(
        matlab_row[f"{matlab_prefix}_y_m"]
    )
    dz = finite_float(cpp_row["position_z_m"]) - finite_float(
        matlab_row[f"{matlab_prefix}_z_m"]
    )
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def seed_position_error_m(
    cpp_row: dict[str, str],
    matlab_row: dict[str, str],
) -> float:
    dx = finite_float(cpp_row["seed_position_x_m"]) - finite_float(
        matlab_row["spp_x_m"]
    )
    dy = finite_float(cpp_row["seed_position_y_m"]) - finite_float(
        matlab_row["spp_y_m"]
    )
    dz = finite_float(cpp_row["seed_position_z_m"]) - finite_float(
        matlab_row["spp_z_m"]
    )
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def velocity_error_mps(
    cpp_row: dict[str, str],
    matlab_row: dict[str, str],
) -> float:
    dvx = finite_float(cpp_row["velocity_x_mps"]) - finite_float(
        matlab_row["fgo_vx_mps"]
    )
    dvy = finite_float(cpp_row["velocity_y_mps"]) - finite_float(
        matlab_row["fgo_vy_mps"]
    )
    dvz = finite_float(cpp_row["velocity_z_mps"]) - finite_float(
        matlab_row["fgo_vz_mps"]
    )
    return math.sqrt(dvx * dvx + dvy * dvy + dvz * dvz)


class TarozPosVelAmbPdcWindowCostParityTest(unittest.TestCase):
    def require_files(self, *paths: Path) -> None:
        missing = [display_path(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest(
                "missing optional taroz window cost parity files: "
                + ", ".join(missing)
            )

    def assert_cost_trace_contract(
        self,
        rows: list[dict[str, str]],
        expected_iterations: int,
        expected_initial_cost: float,
        expected_final_cost: float,
    ) -> None:
        self.assertEqual(len(rows), expected_iterations + 1)
        self.assertGreater(len(rows), 1)
        global_iterations = [int(row["global_iteration"]) for row in rows]
        costs = [finite_float(row["cost"]) for row in rows]

        self.assertEqual(global_iterations, sorted(global_iterations))
        self.assertEqual(len(global_iterations), len(set(global_iterations)))
        self.assertEqual(costs[0], expected_initial_cost)
        self.assertEqual(costs[-1], expected_final_cost)
        self.assertLess(costs[-1], costs[0])
        self.assertTrue(all(cost >= 0.0 for cost in costs))

    def test_window_graph_summary_and_cost_trace_match(self) -> None:
        self.require_files(
            CPP_SUMMARY,
            CPP_COST_TRACE,
            CPP_EPOCH_DEBUG,
            MATLAB_GRAPH,
            MATLAB_COST_TRACE,
            MATLAB_EPOCH_STATE,
        )

        summary = read_json(CPP_SUMMARY)
        cpp_rows = read_rows(CPP_COST_TRACE)
        cpp_epoch_rows = read_rows(CPP_EPOCH_DEBUG)
        matlab_graph = read_rows(MATLAB_GRAPH)[0]
        matlab_rows = read_rows(MATLAB_COST_TRACE)
        matlab_epoch_rows = read_rows(MATLAB_EPOCH_STATE)

        self.assertEqual(int(summary["optimized_epochs"]), int(float(matlab_graph["n"])))
        self.assertEqual(epoch_keys(cpp_epoch_rows), epoch_keys(matlab_epoch_rows))
        cpp_epoch_by_key = keyed_rows(cpp_epoch_rows)
        matlab_epoch_by_key = keyed_rows(matlab_epoch_rows)
        self.assertEqual(len(cpp_epoch_by_key), len(cpp_epoch_rows))
        self.assertEqual(len(matlab_epoch_by_key), len(matlab_epoch_rows))
        self.assertEqual(
            int(summary["fixed_solutions"]),
            int(float(matlab_graph["fixed_epochs"])),
        )
        self.assertEqual(
            int(summary["ambiguity_states"]),
            int(float(matlab_graph["valid_ambiguity_states"])),
        )
        self.assertEqual(
            int(summary["optimized_epochs"]),
            int(float(matlab_graph["valid_position_epochs"])),
        )
        self.assertEqual(
            int(summary["optimized_epochs"]),
            int(float(matlab_graph["valid_velocity_epochs"])),
        )
        if "dump_max_epochs" in matlab_graph:
            self.assertEqual(
                int(summary["max_epochs"]),
                int(float(matlab_graph["dump_max_epochs"])),
            )

        valid_keys: list[tuple[int, int]] = []
        fixed_keys: list[tuple[int, int]] = []
        ratio_diffs: list[float] = []
        position_errors: list[float] = []
        seed_position_errors: list[float] = []
        velocity_errors: list[float] = []
        for key, cpp_epoch in cpp_epoch_by_key.items():
            matlab_epoch = matlab_epoch_by_key[key]
            cpp_status = int(cpp_epoch["status"])
            cpp_fixed = cpp_status == 4
            matlab_fixed = int(float(matlab_epoch["idxfix"])) == 1

            self.assertEqual(cpp_fixed, matlab_fixed, f"fixed status at {key}")
            self.assertEqual(
                int(cpp_epoch["ambiguity_candidates"]),
                int(float(matlab_epoch["candidate_count"])),
                f"ambiguity candidates at {key}",
            )
            self.assertEqual(
                int(cpp_epoch["num_fixed_ambiguities"]),
                int(float(matlab_epoch["fixed_ambiguity_count"])),
                f"fixed ambiguity count at {key}",
            )
            seed_position_errors.append(seed_position_error_m(cpp_epoch, matlab_epoch))

            if cpp_status == 0:
                continue

            valid_keys.append(key)
            if cpp_fixed:
                fixed_keys.append(key)

            ratio_diffs.append(
                abs(finite_float(cpp_epoch["ratio"]) - finite_float(matlab_epoch["ratio"]))
            )
            matlab_position_prefix = "fixed" if cpp_fixed else "float"
            position_errors.append(
                ecef_error_m(cpp_epoch, matlab_epoch, matlab_position_prefix)
            )
            velocity_errors.append(velocity_error_mps(cpp_epoch, matlab_epoch))

        self.assertEqual(len(valid_keys), int(summary["valid_solutions"]))
        self.assertEqual(len(fixed_keys), int(summary["fixed_solutions"]))
        self.assertTrue(ratio_diffs)
        self.assertTrue(position_errors)
        self.assertTrue(seed_position_errors)
        self.assertTrue(velocity_errors)

        self.assertLessEqual(max(seed_position_errors), 2e-6)

        self.assertLessEqual(sum(ratio_diffs) / len(ratio_diffs), 0.02)
        self.assertLessEqual(percentile(ratio_diffs, 0.95), 0.06)
        self.assertLessEqual(max(ratio_diffs), 0.06)

        self.assertLessEqual(sum(position_errors) / len(position_errors), 0.004)
        self.assertLessEqual(percentile(position_errors, 0.95), 0.008)
        self.assertLessEqual(max(position_errors), 0.008)

        self.assertLessEqual(sum(velocity_errors) / len(velocity_errors), 0.003)
        self.assertLessEqual(percentile(velocity_errors, 0.95), 0.008)
        self.assertLessEqual(max(velocity_errors), 0.02)

        cpp_initial_cost = finite_float(summary["initial_cost"])
        cpp_final_cost = finite_float(summary["final_cost"])
        matlab_initial_cost = finite_float(matlab_graph["initial_cost"])
        matlab_final_cost = finite_float(matlab_graph["final_cost"])
        matlab_iterations = int(float(matlab_graph["iterations"]))

        self.assert_cost_trace_contract(
            cpp_rows,
            int(summary["iterations"]),
            cpp_initial_cost,
            cpp_final_cost,
        )
        self.assert_cost_trace_contract(
            matlab_rows,
            matlab_iterations,
            matlab_initial_cost,
            matlab_final_cost,
        )

        self.assertLessEqual(
            abs(int(summary["iterations"]) - matlab_iterations),
            4,
        )
        self.assertLessEqual(relative_error(cpp_initial_cost, matlab_initial_cost), 1.20)
        self.assertLessEqual(relative_error(cpp_final_cost, matlab_final_cost), 0.60)


if __name__ == "__main__":
    unittest.main()
