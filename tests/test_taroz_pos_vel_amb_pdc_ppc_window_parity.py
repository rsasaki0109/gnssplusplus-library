#!/usr/bin/env python3
"""Optional public PPC-Dataset window parity for taroz ambiguity PDC."""

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
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_CPP_DIR",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_first_120_seed_current/nagoya_run3",
)
MATLAB_DIR = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_MATLAB_DIR",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_first_120_debug",
)
CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_first_120_seed_current/nagoya_run3/summary.json",
)
CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_first_120_seed_current/nagoya_run3/epoch_debug.csv",
)
CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_first_120_seed_current/nagoya_run3/cost_trace.csv",
)
MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_first_120_debug/graph_detail.csv",
)
MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_first_120_debug/per_epoch_state.csv",
)
MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_first_120_debug/optimizer_cost_trace.csv",
)
NAGOYA1_SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run1_shifted_120_seed_current/nagoya_run1/summary.json",
)
NAGOYA1_SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run1_shifted_120_seed_current/nagoya_run1/epoch_debug.csv",
)
NAGOYA1_SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run1_shifted_120_seed_current/nagoya_run1/lambda_debug.csv",
)
NAGOYA1_SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run1_shifted_120_seed_current/nagoya_run1/cost_trace.csv",
)
NAGOYA1_SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run1_shifted_120_debug/graph_detail.csv",
)
NAGOYA1_SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run1_shifted_120_debug/per_epoch_state.csv",
)
NAGOYA1_SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run1_shifted_120_debug/per_lambda_detail.csv",
)
NAGOYA1_SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA1_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run1_shifted_120_debug/optimizer_cost_trace.csv",
)
NAGOYA2_SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_shifted_120_seed_current/nagoya_run2/summary.json",
)
NAGOYA2_SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_shifted_120_seed_current/nagoya_run2/epoch_debug.csv",
)
NAGOYA2_SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_shifted_120_seed_current/nagoya_run2/lambda_debug.csv",
)
NAGOYA2_SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run2_shifted_120_seed_current/nagoya_run2/cost_trace.csv",
)
NAGOYA2_SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run2_shifted_120_debug/graph_detail.csv",
)
NAGOYA2_SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run2_shifted_120_debug/per_epoch_state.csv",
)
NAGOYA2_SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run2_shifted_120_debug/per_lambda_detail.csv",
)
NAGOYA2_SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_NAGOYA2_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run2_shifted_120_debug/optimizer_cost_trace.csv",
)
SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current/nagoya_run3/summary.json",
)
SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current/nagoya_run3/epoch_debug.csv",
)
SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current/nagoya_run3/lambda_debug.csv",
)
SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted_120_seed_current/nagoya_run3/cost_trace.csv",
)
SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug/graph_detail.csv",
)
SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug/per_epoch_state.csv",
)
SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug/per_lambda_detail.csv",
)
SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted_120_debug/optimizer_cost_trace.csv",
)
SHIFTED2_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current/nagoya_run3/summary.json",
)
SHIFTED2_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current/nagoya_run3/epoch_debug.csv",
)
SHIFTED2_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current/nagoya_run3/lambda_debug.csv",
)
SHIFTED2_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_nagoya_run3_shifted2_120_seed_current/nagoya_run3/cost_trace.csv",
)
SHIFTED2_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug/graph_detail.csv",
)
SHIFTED2_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug/per_epoch_state.csv",
)
SHIFTED2_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug/per_lambda_detail.csv",
)
SHIFTED2_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_SHIFTED2_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_nagoya_run3_shifted2_120_debug/optimizer_cost_trace.csv",
)
TOKYO_SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current/tokyo_run1/summary.json",
)
TOKYO_SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current/tokyo_run1/epoch_debug.csv",
)
TOKYO_SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current/tokyo_run1/lambda_debug.csv",
)
TOKYO_SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run1_shifted_120_seed_current/tokyo_run1/cost_trace.csv",
)
TOKYO_SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug/graph_detail.csv",
)
TOKYO_SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug/per_epoch_state.csv",
)
TOKYO_SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug/per_lambda_detail.csv",
)
TOKYO_SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run1_shifted_120_debug/optimizer_cost_trace.csv",
)
TOKYO2_SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current/tokyo_run2/summary.json",
)
TOKYO2_SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current/tokyo_run2/epoch_debug.csv",
)
TOKYO2_SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current/tokyo_run2/lambda_debug.csv",
)
TOKYO2_SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run2_shifted_120_seed_current/tokyo_run2/cost_trace.csv",
)
TOKYO2_SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug/graph_detail.csv",
)
TOKYO2_SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug/per_epoch_state.csv",
)
TOKYO2_SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug/per_lambda_detail.csv",
)
TOKYO2_SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO2_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run2_shifted_120_debug/optimizer_cost_trace.csv",
)
TOKYO3_SHIFTED_CPP_SUMMARY = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_CPP_SUMMARY",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current/tokyo_run3/summary.json",
)
TOKYO3_SHIFTED_CPP_EPOCH_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_CPP_EPOCH_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current/tokyo_run3/epoch_debug.csv",
)
TOKYO3_SHIFTED_CPP_LAMBDA_DEBUG = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_CPP_LAMBDA_DEBUG",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current/tokyo_run3/lambda_debug.csv",
)
TOKYO3_SHIFTED_CPP_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_CPP_COST_TRACE",
    "output/dogfood/ppc_taroz_amb_pdc_tokyo_run3_shifted_120_seed_current/tokyo_run3/cost_trace.csv",
)
TOKYO3_SHIFTED_MATLAB_GRAPH = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_MATLAB_GRAPH",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug/graph_detail.csv",
)
TOKYO3_SHIFTED_MATLAB_EPOCH_STATE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_MATLAB_EPOCH_STATE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug/per_epoch_state.csv",
)
TOKYO3_SHIFTED_MATLAB_LAMBDA_DETAIL = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_MATLAB_LAMBDA_DETAIL",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug/per_lambda_detail.csv",
)
TOKYO3_SHIFTED_MATLAB_COST_TRACE = configured_path(
    "GNSSPP_TAROZ_POS_VEL_AMB_PDC_PPC_TOKYO3_SHIFTED_MATLAB_COST_TRACE",
    "output/dogfood/taroz_matlab_pos_vel_amb_pdc_ppc_tokyo_run3_shifted_120_debug/optimizer_cost_trace.csv",
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


def finite_float(value: object) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"not finite: {value!r}")
    return result


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = math.ceil(fraction * len(ordered)) - 1
    return ordered[max(0, min(index, len(ordered) - 1))]


def epoch_key(row: dict[str, str]) -> tuple[int, int]:
    return (int(float(row["gps_week"])), round(float(row["gps_tow"]) * 1000))


def ecef_error_m(cpp_row: dict[str, str], matlab_row: dict[str, str]) -> float:
    return math.dist(
        (
            finite_float(cpp_row["position_x_m"]),
            finite_float(cpp_row["position_y_m"]),
            finite_float(cpp_row["position_z_m"]),
        ),
        (
            finite_float(matlab_row["float_x_m"]),
            finite_float(matlab_row["float_y_m"]),
            finite_float(matlab_row["float_z_m"]),
        ),
    )


def fixed_or_float_ecef_error_m(
    cpp_row: dict[str, str],
    matlab_row: dict[str, str],
) -> float:
    prefix = "fixed" if int(float(matlab_row["idxfix"])) == 1 else "float"
    return math.dist(
        (
            finite_float(cpp_row["position_x_m"]),
            finite_float(cpp_row["position_y_m"]),
            finite_float(cpp_row["position_z_m"]),
        ),
        (
            finite_float(matlab_row[f"{prefix}_x_m"]),
            finite_float(matlab_row[f"{prefix}_y_m"]),
            finite_float(matlab_row[f"{prefix}_z_m"]),
        ),
    )


def seed_error_m(cpp_row: dict[str, str], matlab_row: dict[str, str]) -> float:
    return math.dist(
        (
            finite_float(cpp_row["seed_position_x_m"]),
            finite_float(cpp_row["seed_position_y_m"]),
            finite_float(cpp_row["seed_position_z_m"]),
        ),
        (
            finite_float(matlab_row["spp_x_m"]),
            finite_float(matlab_row["spp_y_m"]),
            finite_float(matlab_row["spp_z_m"]),
        ),
    )


def velocity_error_mps(cpp_row: dict[str, str], matlab_row: dict[str, str]) -> float:
    return math.dist(
        (
            finite_float(cpp_row["velocity_x_mps"]),
            finite_float(cpp_row["velocity_y_mps"]),
            finite_float(cpp_row["velocity_z_mps"]),
        ),
        (
            finite_float(matlab_row["fgo_vx_mps"]),
            finite_float(matlab_row["fgo_vy_mps"]),
            finite_float(matlab_row["fgo_vz_mps"]),
        ),
    )


def lambda_key(row: dict[str, str]) -> tuple[int, int, int, int, str, str]:
    return (
        int(float(row["gps_week"])),
        round(float(row["gps_tow"]) * 1000),
        int(row["row"]),
        int(row["col"]),
        row["satellite"],
        row["other_satellite"],
    )


class TarozPosVelAmbPdcPpcWindowParityTest(unittest.TestCase):
    def require_files(self, *paths: Path) -> None:
        missing = [display_path(path) for path in paths if not path.exists()]
        if missing:
            self.skipTest(
                "missing optional public PPC taroz parity files: "
                + ", ".join(missing)
            )

    def assert_public_cost_trace_contract(
        self,
        *,
        cpp_rows: list[dict[str, str]],
        matlab_rows: list[dict[str, str]],
        summary: dict[str, object],
        matlab_graph: dict[str, str],
        cost_reduction_max: float,
    ) -> None:
        self.assertGreaterEqual(len(cpp_rows), 2)
        self.assertGreaterEqual(len(matlab_rows), 2)
        self.assertLessEqual(len(cpp_rows), int(summary["iterations"]) + 1)
        self.assertLessEqual(
            len(matlab_rows),
            int(float(matlab_graph["iterations"])) + 1,
        )

        for rows, label in ((cpp_rows, "C++"), (matlab_rows, "MATLAB")):
            global_iterations = [int(row["global_iteration"]) for row in rows]
            costs = [finite_float(row["cost"]) for row in rows]
            self.assertEqual(global_iterations, sorted(global_iterations), label)
            self.assertEqual(len(global_iterations), len(set(global_iterations)), label)
            self.assertTrue(all(cost >= 0.0 for cost in costs), label)
            self.assertLess(costs[-1], costs[0], label)

        cpp_initial_cost = finite_float(summary["initial_cost"])
        cpp_final_cost = finite_float(summary["final_cost"])
        matlab_initial_cost = finite_float(matlab_graph["initial_cost"])
        matlab_final_cost = finite_float(matlab_graph["final_cost"])
        cpp_costs = [finite_float(row["cost"]) for row in cpp_rows]
        matlab_costs = [finite_float(row["cost"]) for row in matlab_rows]

        self.assertAlmostEqual(cpp_costs[0], cpp_initial_cost, places=3)
        self.assertAlmostEqual(cpp_costs[-1], cpp_final_cost, places=3)
        self.assertAlmostEqual(matlab_costs[0], matlab_initial_cost, places=3)
        self.assertAlmostEqual(matlab_costs[-1], matlab_final_cost, places=3)

        cpp_reduction = cpp_final_cost / cpp_initial_cost
        matlab_reduction = matlab_final_cost / matlab_initial_cost
        self.assertLessEqual(cpp_reduction, cost_reduction_max)
        self.assertLessEqual(matlab_reduction, cost_reduction_max)

        initial_scale = cpp_initial_cost / matlab_initial_cost
        final_scale = cpp_final_cost / matlab_final_cost
        reduction_scale = cpp_reduction / matlab_reduction
        self.assertGreaterEqual(initial_scale, 1.8)
        self.assertLessEqual(initial_scale, 2.2)
        self.assertGreaterEqual(final_scale, 1.2)
        self.assertLessEqual(final_scale, 2.0)
        self.assertGreaterEqual(reduction_scale, 0.6)
        self.assertLessEqual(reduction_scale, 1.0)

    def assert_public_shifted_window_parity(
        self,
        *,
        cpp_summary_path: Path,
        cpp_epoch_debug_path: Path,
        cpp_lambda_debug_path: Path,
        cpp_cost_trace_path: Path,
        matlab_graph_path: Path,
        matlab_epoch_state_path: Path,
        matlab_lambda_detail_path: Path,
        matlab_cost_trace_path: Path,
        expected_fixed_epochs: int,
        position_mean_max_m: float,
        position_p95_max_m: float,
        position_max_m: float,
        ambiguity_float_p95_max_cycles: float,
        ambiguity_float_max_cycles: float,
        ratio_mean_max: float = 0.012,
        ratio_p95_max: float = 0.014,
        ratio_max: float = 0.02,
        lambda_ratio_p95_max: float = 0.014,
        lambda_ratio_max: float = 0.02,
        covariance_p95_max: float = 6e-6,
        covariance_max: float = 1.2e-5,
        cost_reduction_max: float = 2e-5,
    ) -> None:
        self.require_files(
            cpp_summary_path,
            cpp_epoch_debug_path,
            cpp_lambda_debug_path,
            cpp_cost_trace_path,
            matlab_graph_path,
            matlab_epoch_state_path,
            matlab_lambda_detail_path,
            matlab_cost_trace_path,
        )

        summary = read_json(cpp_summary_path)
        cpp_rows = read_rows(cpp_epoch_debug_path)
        cpp_lambda_rows = read_rows(cpp_lambda_debug_path)
        cpp_cost_rows = read_rows(cpp_cost_trace_path)
        matlab_graph = read_rows(matlab_graph_path)[0]
        matlab_rows = read_rows(matlab_epoch_state_path)
        matlab_lambda_rows = read_rows(matlab_lambda_detail_path)
        matlab_cost_rows = read_rows(matlab_cost_trace_path)

        self.assertEqual(summary["optimized_epochs"], 120)
        self.assertEqual(summary["valid_solutions"], 120)
        self.assertEqual(summary["fixed_solutions"], expected_fixed_epochs)
        self.assertEqual(summary["float_solutions"], 120 - expected_fixed_epochs)
        self.assertEqual(summary["seed_matched_epochs"], 120)
        self.assertEqual(summary["seed_interpolated_epochs"], 0)
        self.assertEqual(int(float(matlab_graph["n"])), 120)
        self.assertEqual(int(float(matlab_graph["fixed_epochs"])), expected_fixed_epochs)
        self.assertEqual(int(float(matlab_graph["valid_position_epochs"])), 120)
        self.assertEqual(int(float(matlab_graph["valid_velocity_epochs"])), 120)
        self.assert_public_cost_trace_contract(
            cpp_rows=cpp_cost_rows,
            matlab_rows=matlab_cost_rows,
            summary=summary,
            matlab_graph=matlab_graph,
            cost_reduction_max=cost_reduction_max,
        )

        self.assertEqual(
            [epoch_key(row) for row in cpp_rows],
            [epoch_key(row) for row in matlab_rows],
        )

        position_errors: list[float] = []
        seed_errors: list[float] = []
        velocity_errors: list[float] = []
        ratio_diffs: list[float] = []
        candidate_count_diffs: list[int] = []
        cpp_fixed_epochs = 0
        matlab_fixed_epochs = 0
        for cpp_row, matlab_row in zip(cpp_rows, matlab_rows):
            cpp_fixed_epochs += int(cpp_row["status"]) == 4
            matlab_fixed_epochs += int(float(matlab_row["idxfix"])) == 1
            self.assertEqual(
                int(cpp_row["status"]) == 4,
                int(float(matlab_row["idxfix"])) == 1,
            )
            self.assertEqual(
                int(cpp_row["num_fixed_ambiguities"]),
                int(float(matlab_row["fixed_ambiguity_count"])),
            )
            position_errors.append(fixed_or_float_ecef_error_m(cpp_row, matlab_row))
            seed_errors.append(seed_error_m(cpp_row, matlab_row))
            velocity_errors.append(velocity_error_mps(cpp_row, matlab_row))
            ratio_diffs.append(
                abs(finite_float(cpp_row["ratio"]) - finite_float(matlab_row["ratio"]))
            )
            candidate_count_diffs.append(
                abs(
                    int(cpp_row["ambiguity_candidates"])
                    - int(float(matlab_row["candidate_count"]))
                )
            )

        self.assertEqual(cpp_fixed_epochs, expected_fixed_epochs)
        self.assertEqual(matlab_fixed_epochs, expected_fixed_epochs)
        self.assertLessEqual(max(seed_errors), 1e-6)
        self.assertLessEqual(sum(position_errors) / len(position_errors), position_mean_max_m)
        self.assertLessEqual(percentile(position_errors, 0.95), position_p95_max_m)
        self.assertLessEqual(max(position_errors), position_max_m)
        self.assertLessEqual(sum(velocity_errors) / len(velocity_errors), 0.001)
        self.assertLessEqual(percentile(velocity_errors, 0.95), 0.002)
        self.assertLessEqual(max(velocity_errors), 0.008)
        self.assertEqual(sum(candidate_count_diffs), 0)
        self.assertLessEqual(sum(ratio_diffs) / len(ratio_diffs), ratio_mean_max)
        self.assertLessEqual(percentile(ratio_diffs, 0.95), ratio_p95_max)
        self.assertLessEqual(max(ratio_diffs), ratio_max)

        matlab_lambda_by_key = {lambda_key(row): row for row in matlab_lambda_rows}
        cpp_lambda_keys = {lambda_key(row) for row in cpp_lambda_rows}
        self.assertEqual(len(matlab_lambda_by_key), len(matlab_lambda_rows))
        self.assertEqual(cpp_lambda_keys, set(matlab_lambda_by_key))

        ambiguity_float_diffs: list[float] = []
        covariance_diffs: list[float] = []
        lambda_ratio_diffs: list[float] = []
        for cpp_row in cpp_lambda_rows:
            matlab_row = matlab_lambda_by_key[lambda_key(cpp_row)]
            self.assertEqual(int(cpp_row["solved"]), int(float(matlab_row["solved"])))
            self.assertEqual(
                int(cpp_row["fixed_epoch"]),
                int(float(matlab_row["fixed_epoch"])),
            )
            self.assertEqual(
                int(cpp_row["candidate_count"]),
                int(float(matlab_row["candidate_count"])),
            )
            ambiguity_float_diffs.append(
                abs(
                    finite_float(cpp_row["ambiguity_float"])
                    - finite_float(matlab_row["ambiguity_float"])
                )
            )
            covariance_diffs.append(
                abs(
                    finite_float(cpp_row["covariance"])
                    - finite_float(matlab_row["covariance"])
                )
            )
            lambda_ratio_diffs.append(
                abs(finite_float(cpp_row["ratio"]) - finite_float(matlab_row["ratio"]))
            )

        self.assertLessEqual(
            percentile(ambiguity_float_diffs, 0.95),
            ambiguity_float_p95_max_cycles,
        )
        self.assertLessEqual(max(ambiguity_float_diffs), ambiguity_float_max_cycles)
        self.assertLessEqual(percentile(covariance_diffs, 0.95), covariance_p95_max)
        self.assertLessEqual(max(covariance_diffs), covariance_max)
        self.assertLessEqual(percentile(lambda_ratio_diffs, 0.95), lambda_ratio_p95_max)
        self.assertLessEqual(max(lambda_ratio_diffs), lambda_ratio_max)

    def test_public_ppc_nagoya_run3_first_120_window(self) -> None:
        self.require_files(
            CPP_SUMMARY,
            CPP_EPOCH_DEBUG,
            CPP_COST_TRACE,
            MATLAB_GRAPH,
            MATLAB_EPOCH_STATE,
            MATLAB_COST_TRACE,
        )

        summary = read_json(CPP_SUMMARY)
        cpp_rows = read_rows(CPP_EPOCH_DEBUG)
        cpp_cost = read_rows(CPP_COST_TRACE)
        matlab_graph = read_rows(MATLAB_GRAPH)[0]
        matlab_rows = read_rows(MATLAB_EPOCH_STATE)
        matlab_cost = read_rows(MATLAB_COST_TRACE)

        self.assertEqual(summary["optimized_epochs"], 120)
        self.assertEqual(summary["valid_solutions"], 120)
        self.assertEqual(summary["fixed_solutions"], 0)
        self.assertEqual(summary["float_solutions"], 120)
        self.assertEqual(summary["seed_matched_epochs"], 120)
        self.assertEqual(summary["seed_interpolated_epochs"], 0)
        self.assertEqual(int(float(matlab_graph["n"])), 120)
        self.assertEqual(int(float(matlab_graph["fixed_epochs"])), 0)
        self.assertEqual(int(float(matlab_graph["valid_position_epochs"])), 120)
        self.assertEqual(int(float(matlab_graph["valid_velocity_epochs"])), 120)

        self.assertEqual(
            [epoch_key(row) for row in cpp_rows],
            [epoch_key(row) for row in matlab_rows],
        )
        self.assertGreaterEqual(len(cpp_cost), 2)
        self.assertGreaterEqual(len(matlab_cost), 2)
        self.assertLessEqual(len(cpp_cost), int(summary["iterations"]) + 1)
        self.assertLessEqual(len(matlab_cost), int(float(matlab_graph["iterations"])) + 1)
        self.assertEqual(
            [int(row["global_iteration"]) for row in cpp_cost],
            sorted(int(row["global_iteration"]) for row in cpp_cost),
        )
        self.assertEqual(
            [int(row["global_iteration"]) for row in matlab_cost],
            sorted(int(row["global_iteration"]) for row in matlab_cost),
        )
        self.assertAlmostEqual(
            finite_float(cpp_cost[0]["cost"]),
            finite_float(summary["initial_cost"]),
            places=3,
        )
        self.assertAlmostEqual(
            finite_float(cpp_cost[-1]["cost"]),
            finite_float(summary["final_cost"]),
            places=3,
        )
        self.assertAlmostEqual(
            finite_float(matlab_cost[0]["cost"]),
            finite_float(matlab_graph["initial_cost"]),
            places=3,
        )
        self.assertAlmostEqual(
            finite_float(matlab_cost[-1]["cost"]),
            finite_float(matlab_graph["final_cost"]),
            places=3,
        )
        self.assertLess(
            finite_float(cpp_cost[-1]["cost"]),
            finite_float(cpp_cost[0]["cost"]),
        )
        self.assertLess(
            finite_float(matlab_cost[-1]["cost"]),
            finite_float(matlab_cost[0]["cost"]),
        )

        position_errors: list[float] = []
        seed_errors: list[float] = []
        velocity_errors: list[float] = []
        ratio_diffs: list[float] = []
        candidate_count_diffs: list[int] = []
        for cpp_row, matlab_row in zip(cpp_rows, matlab_rows):
            self.assertEqual(int(cpp_row["status"]), 3)
            self.assertEqual(int(float(matlab_row["idxfix"])), 0)
            self.assertEqual(int(cpp_row["num_fixed_ambiguities"]), 0)
            self.assertEqual(int(float(matlab_row["fixed_ambiguity_count"])), 0)

            position_errors.append(ecef_error_m(cpp_row, matlab_row))
            seed_errors.append(seed_error_m(cpp_row, matlab_row))
            velocity_errors.append(velocity_error_mps(cpp_row, matlab_row))
            ratio_diffs.append(
                abs(finite_float(cpp_row["ratio"]) - finite_float(matlab_row["ratio"]))
            )
            candidate_count_diffs.append(
                abs(
                    int(cpp_row["ambiguity_candidates"])
                    - int(float(matlab_row["candidate_count"]))
                )
            )

        self.assertLessEqual(max(seed_errors), 1e-6)
        self.assertLessEqual(sum(position_errors) / len(position_errors), 0.28)
        self.assertLessEqual(percentile(position_errors, 0.95), 0.34)
        self.assertLessEqual(max(position_errors), 0.35)
        self.assertLessEqual(sum(velocity_errors) / len(velocity_errors), 0.021)
        self.assertLessEqual(percentile(velocity_errors, 0.95), 0.046)
        self.assertLessEqual(max(velocity_errors), 0.055)
        self.assertLessEqual(sum(ratio_diffs) / len(ratio_diffs), 0.09)
        self.assertLessEqual(percentile(ratio_diffs, 0.95), 0.19)
        self.assertLessEqual(max(ratio_diffs), 0.27)
        self.assertLessEqual(sum(candidate_count_diffs) / len(candidate_count_diffs), 0.4)
        self.assertLessEqual(max(candidate_count_diffs), 1)

    def test_public_ppc_nagoya_run1_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=NAGOYA1_SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=NAGOYA1_SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=NAGOYA1_SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=NAGOYA1_SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=NAGOYA1_SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=NAGOYA1_SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=NAGOYA1_SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=NAGOYA1_SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=116,
            position_mean_max_m=0.001,
            position_p95_max_m=0.001,
            position_max_m=0.001,
            ambiguity_float_p95_max_cycles=0.002,
            ambiguity_float_max_cycles=0.002,
            covariance_p95_max=6.5e-6,
        )

    def test_public_ppc_nagoya_run2_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=NAGOYA2_SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=NAGOYA2_SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=NAGOYA2_SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=NAGOYA2_SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=NAGOYA2_SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=NAGOYA2_SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=NAGOYA2_SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=NAGOYA2_SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=39,
            position_mean_max_m=0.001,
            position_p95_max_m=0.001,
            position_max_m=0.001,
            ambiguity_float_p95_max_cycles=0.002,
            ambiguity_float_max_cycles=0.002,
        )

    def test_public_ppc_nagoya_run3_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=112,
            position_mean_max_m=0.001,
            position_p95_max_m=0.002,
            position_max_m=0.004,
            ambiguity_float_p95_max_cycles=0.005,
            ambiguity_float_max_cycles=0.006,
        )

    def test_public_ppc_nagoya_run3_shifted2_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=SHIFTED2_CPP_SUMMARY,
            cpp_epoch_debug_path=SHIFTED2_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=SHIFTED2_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=SHIFTED2_CPP_COST_TRACE,
            matlab_graph_path=SHIFTED2_MATLAB_GRAPH,
            matlab_epoch_state_path=SHIFTED2_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=SHIFTED2_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=SHIFTED2_MATLAB_COST_TRACE,
            expected_fixed_epochs=29,
            position_mean_max_m=0.003,
            position_p95_max_m=0.004,
            position_max_m=0.005,
            ambiguity_float_p95_max_cycles=0.008,
            ambiguity_float_max_cycles=0.008,
        )

    def test_public_ppc_tokyo_run1_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=TOKYO_SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=TOKYO_SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=TOKYO_SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=TOKYO_SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=TOKYO_SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=TOKYO_SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=TOKYO_SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=TOKYO_SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=108,
            position_mean_max_m=0.001,
            position_p95_max_m=0.002,
            position_max_m=0.002,
            ambiguity_float_p95_max_cycles=0.005,
            ambiguity_float_max_cycles=0.006,
            ratio_mean_max=0.012,
            ratio_p95_max=0.04,
            ratio_max=0.05,
            lambda_ratio_p95_max=0.04,
            lambda_ratio_max=0.05,
        )

    def test_public_ppc_tokyo_run2_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=TOKYO2_SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=TOKYO2_SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=TOKYO2_SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=TOKYO2_SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=TOKYO2_SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=TOKYO2_SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=TOKYO2_SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=TOKYO2_SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=115,
            position_mean_max_m=0.001,
            position_p95_max_m=0.001,
            position_max_m=0.001,
            ambiguity_float_p95_max_cycles=0.002,
            ambiguity_float_max_cycles=0.002,
            ratio_mean_max=0.012,
            ratio_p95_max=0.04,
            ratio_max=0.09,
            lambda_ratio_p95_max=0.04,
            lambda_ratio_max=0.09,
            cost_reduction_max=4e-5,
        )

    def test_public_ppc_tokyo_run3_shifted_120_window_parity(self) -> None:
        self.assert_public_shifted_window_parity(
            cpp_summary_path=TOKYO3_SHIFTED_CPP_SUMMARY,
            cpp_epoch_debug_path=TOKYO3_SHIFTED_CPP_EPOCH_DEBUG,
            cpp_lambda_debug_path=TOKYO3_SHIFTED_CPP_LAMBDA_DEBUG,
            cpp_cost_trace_path=TOKYO3_SHIFTED_CPP_COST_TRACE,
            matlab_graph_path=TOKYO3_SHIFTED_MATLAB_GRAPH,
            matlab_epoch_state_path=TOKYO3_SHIFTED_MATLAB_EPOCH_STATE,
            matlab_lambda_detail_path=TOKYO3_SHIFTED_MATLAB_LAMBDA_DETAIL,
            matlab_cost_trace_path=TOKYO3_SHIFTED_MATLAB_COST_TRACE,
            expected_fixed_epochs=113,
            position_mean_max_m=0.001,
            position_p95_max_m=0.001,
            position_max_m=0.001,
            ambiguity_float_p95_max_cycles=0.002,
            ambiguity_float_max_cycles=0.002,
            ratio_mean_max=0.01,
            ratio_p95_max=0.03,
            ratio_max=0.04,
            lambda_ratio_p95_max=0.03,
            lambda_ratio_max=0.04,
        )


if __name__ == "__main__":
    unittest.main()
