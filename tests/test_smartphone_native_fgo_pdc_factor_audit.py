#!/usr/bin/env python3
"""Contract tests for the truth-free native-FGO PDC factor audit."""

from __future__ import annotations

import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "apps" / "commands" / "benchmarks"))
import gnss_smartphone_native_fgo_pdc_factor_audit as audit  # noqa: E402


def write_factor_csv(path: Path, *, rank_deficient: bool = False) -> None:
    fields = [
        "epoch_index", "clock_group", "sigma_p_m", "los_x", "los_y", "los_z",
        "initial_res_pc_m", "final_residual_m",
    ]
    directions = [
        (0.8, 0.2, 0.55),
        (-0.3, 0.9, 0.25),
        (0.4, -0.5, 0.77),
        (-0.7, -0.2, 0.68),
        (0.1, 0.6, -0.79),
        (-0.55, 0.4, -0.73),
    ]
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for epoch in range(2):
            for index, direction in enumerate(directions):
                if rank_deficient:
                    direction = directions[0]
                writer.writerow({
                    "epoch_index": epoch,
                    "clock_group": 2,
                    "sigma_p_m": 3.0 + 0.1 * index,
                    "los_x": direction[0],
                    "los_y": direction[1],
                    "los_z": direction[2],
                    "initial_res_pc_m": 2.0,
                    "final_residual_m": 0.2,
                })


def valid_summary() -> dict[str, object]:
    return {
        "backend": "eigen", "preset": "default", "input_epochs": 2,
        "optimized_epochs": 2, "valid_solutions": 2,
        "pseudorange_factors": 12, "undifferenced_doppler_factors": 6,
        "tdcp_factors": 4, "carrier_phase_factors": 2,
        "ambiguity_states": 1, "motion_factors": 1, "graph_factors": 30,
        "graph_values": 12, "initial_cost": 100.0, "final_cost": 5.0,
        "iterations": 7, "converged": True, "last_update_norm_m": 1e-5,
        "residual_rms_m": 2.0, "tdcp_residual_rms_m": 0.02,
        "undifferenced_doppler_residual_rms_mps": 1.5,
        "carrier_phase_residual_rms_m": 0.01,
        "doppler_velocity_wls_valid_epochs": 2,
        "doppler_velocity_wls_rejected_epochs": 0,
        "doppler_velocity_wls_max_velocity_norm_mps": 10.0,
        "doppler_velocity_wls_max_normalized_rms": 2.0,
        "single_difference_doppler_factors": 0,
        "single_difference_tdcp_factors": 0,
    }


class NativeFgoPdcFactorAuditTests(unittest.TestCase):
    def test_stage_matrix_is_immutable_and_command_has_no_base(self) -> None:
        self.assertEqual(len(audit.STAGES), 10)
        command = audit.command_for_stage(Path("gnss_fgo"), audit.STAGES[3], Path("out"))
        self.assertIn("--corrected-undifferenced-doppler-factors", command)
        self.assertIn("--doppler-velocity-wls-initialization", command)
        self.assertIn("--velocity-motion-factors", command)
        self.assertIn("--no-dd-factors", command)
        self.assertNotIn("--base", command)
        self.assertEqual(command, audit.command_for_stage(Path("gnss_fgo"), audit.STAGES[3], Path("out")))

    def test_realistic_ecef_scale_jacobian_is_finite_and_ranked(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "factor.csv"
            write_factor_csv(path)
            result = audit.weighted_p_jacobian_metrics(path)
        self.assertTrue(result["available"])
        self.assertEqual(result["rows"], 12)
        self.assertEqual(result["columns"], 9)
        # With no absolute clock prior the per-epoch clock and inter-system
        # bias columns have one observable gauge direction.  The audit must
        # expose that rank fact rather than claiming a full-rank solve.
        self.assertEqual(result["rank"], 8)
        self.assertFalse(result["full_column_rank"])
        self.assertTrue(np.isfinite(float(result["condition_number"])))
        self.assertTrue(all(np.isfinite(value) for value in result["column_norms"]))
        self.assertAlmostEqual(float(result["whitened_final_residual_rms"]), 0.064, places=2)

    def test_rank_deficiency_is_reported_not_hidden(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "factor.csv"
            write_factor_csv(path, rank_deficient=True)
            result = audit.weighted_p_jacobian_metrics(path)
        self.assertFalse(result["full_column_rank"])
        self.assertLess(int(result["rank"]), int(result["columns"]))

    def test_stage_contract_rejects_cost_increase_and_base_rows(self) -> None:
        summary = valid_summary()
        stage = audit.STAGES[1]
        self.assertIs(audit._summary_contract(summary, stage), summary)
        increased = dict(summary)
        increased["final_cost"] = 101.0
        with self.assertRaises(audit.FactorAuditError):
            audit._summary_contract(increased, stage)
        forbidden = dict(summary)
        forbidden["single_difference_doppler_factors"] = 1
        with self.assertRaises(audit.FactorAuditError):
            audit._summary_contract(forbidden, stage)

    def test_state_metrics_keeps_clock_and_ambiguity_unknown(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "epoch.csv"
            fields = [
                "epoch_index", "gps_tow", "position_x_m", "position_y_m", "position_z_m",
                "seed_position_x_m", "seed_position_y_m", "seed_position_z_m",
                "velocity_x_mps", "velocity_y_mps", "velocity_z_mps",
            ]
            with path.open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=fields)
                writer.writeheader()
                writer.writerow({"epoch_index": 0, "gps_tow": 1, "position_x_m": 6000000, "position_y_m": 0, "position_z_m": 0, "seed_position_x_m": 5999999, "seed_position_y_m": 0, "seed_position_z_m": 0, "velocity_x_mps": 1, "velocity_y_mps": 0, "velocity_z_mps": 0})
                writer.writerow({"epoch_index": 1, "gps_tow": 2, "position_x_m": 6000001, "position_y_m": 0, "position_z_m": 0, "seed_position_x_m": 6000000, "seed_position_y_m": 0, "seed_position_z_m": 0, "velocity_x_mps": 1, "velocity_y_mps": 0, "velocity_z_mps": 0})
            result = audit._state_metrics(path, valid_summary())
        self.assertEqual(result["continuity"]["over_70mps_transition_count"], 0)
        self.assertFalse(result["clock_bias_m"]["available"])
        self.assertFalse(result["clock_drift_mps"]["available"])
        self.assertFalse(result["ambiguity_m"]["available"])


if __name__ == "__main__":
    unittest.main()
