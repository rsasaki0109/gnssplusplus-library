#!/usr/bin/env python3
"""Focused truth-free contract tests for the Phase39 velocity-only candidate."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
SOURCE = ROOT / "apps" / "native" / "gnss_fgo_imu_no_base.cpp"
RESULT = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_phase39_gnss_first_velocity_only_handoff_result_v1.json"
FREEZE = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_phase39_gnss_first_velocity_only_handoff_freeze_v1.json"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase39_velocity_only_handoff as phase39  # noqa: E402


class SmartphonePhase39VelocityOnlyHandoffTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.source = SOURCE.read_text(encoding="utf-8")
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_freeze_routes_and_gates_are_predeclared(self) -> None:
        freeze = phase39.verify_freeze()
        self.assertEqual(freeze["phase"], 39)
        self.assertEqual(freeze["status"], "frozen-before-implementation-and-solver")
        self.assertEqual(tuple(freeze["objective"]["structural_routes"]), phase39.ROUTES)
        self.assertEqual(
            freeze["objective"]["candidate_flag"],
            phase39.CANDIDATE_FLAG,
        )
        self.assertEqual(
            freeze["predeclared_numeric_gates"]["velocity_norm_max_mps"],
            70.0,
        )
        self.assertEqual(freeze["predeclared_numeric_gates"]["repeat_runs"], 2)

    def test_candidate_command_is_explicit_and_raw_only(self) -> None:
        paths = {
            "device_gnss": Path("/tmp/device_gnss.csv"),
            "device_imu": Path("/tmp/device_imu.csv"),
            "broadcast_nav": Path("/tmp/brdc.nav"),
        }
        candidate = phase39.native_command(paths, "route/pixel5", Path("/tmp/run1"), True)
        flag_off = phase39.native_command(paths, "route/pixel5", Path("/tmp/run2"), False)
        self.assertIn(phase39.CANDIDATE_FLAG, candidate)
        self.assertNotIn(phase39.CANDIDATE_FLAG, flag_off)
        joined = " ".join(candidate).lower()
        self.assertNotIn("ground_truth", joined)
        self.assertNotIn("validation", joined)
        self.assertNotIn("holdout", joined)
        self.assertNotIn(".mat", joined)

    def test_velocity_only_helper_uses_raw_origin_and_does_not_call_legacy_handoff(self) -> None:
        start = self.source.index("bool validateGnssFirstVelocityOnlyHandoff(")
        end = self.source.index("struct RawUtcOutputReport", start)
        helper = self.source[start:end]
        # The synthetic contract represented here is the Phase38 failure mode:
        # GNSS-first positions may be finite but out of Earth, while the raw
        # SPP first position is Earth-valid and the Doppler velocity sequence
        # is finite.  The implementation must therefore convert at the raw
        # origin and must not delegate to the legacy GNSS-position-origin path.
        self.assertIn(
            "const libgnss::Vector3d raw_origin = problem.epochs.front().position_ecef;",
            helper,
        )
        self.assertNotIn("deriveGnssFirstVelocities(", helper)
        self.assertIn("velocities_enu[index] = libgnss::ecef2enu(", helper)
        self.assertIn("report.gnss_first_positions_clocks_copied = 0U;", helper)
        self.assertIn("gnss_first_position_out_of_earth_count", helper)
        self.assertIn("velocity_handoff_source", self.source)
        self.assertIn("gnss-first-optimizer-result", self.source)
        self.assertIn("velocity_initializer", self.source)
        self.assertIn("raw-doppler-wls", self.source)
        self.assertIn("doppler_velocity_wls_edge_hold_max_s = 0.0", self.source)

    def test_summary_validator_rejects_position_clock_copy(self) -> None:
        summary = {
            "dataset_id": "route/pixel5",
            "truth_used": False,
            "production_default_changed": False,
            "native_pdc_imu_tdcp_no_bridge": True,
            "native_quality_anchor": True,
            "native_pdc_state_bridge": False,
            "gnss_first": {
                "handoff_mode": "velocity-only",
                "converged": True,
                "epochs": 2,
                "velocity_states_exported": 2,
                "velocity_valid_count": 2,
                "velocity_nonfinite_count": 0,
                "velocity_over_70_mps_count": 0,
                "max_velocity_norm_mps": 10.0,
                "velocity_handoff_source": "gnss-first-optimizer-result",
                "velocity_initializer": "raw-doppler-wls",
                "velocity_initializer_propagated_count": 0,
                "velocity_initializer_edge_hold_count": 0,
                "velocity_initializer_edge_hold_max_s": 0.0,
                "original_raw_seed_position_count": 2,
                "original_raw_seed_position_invalid_count": 0,
                "positions_clocks_copied": 1,
                "position_invalid_count": 2,
                "clock_invalid_count": 0,
            },
            "graph": {"converged": True, "initial_cost": 2.0, "final_cost": 1.0, "imu_intervals": 1},
            "epochs": {"problem": 2, "output": 2, "pseudorange_factors": 1},
            "tdcp_contract": {"factors_built": 1, "factors_inserted": 1, "nonfinite_residuals": 0},
            "raw_utc_key_contract": {
                "raw_epoch_keys": 2,
                "target_epochs": 1,
                "exact_solution_epochs": 1,
                "interpolated_epochs": 0,
                "edge_hold_epochs": 0,
                "unresolved_epochs": 0,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary), encoding="utf-8")
            with self.assertRaises(phase39.Phase39Error):
                phase39.validate_candidate_summary(path, "route/pixel5", 2)

    def test_sealed_result_records_fail_closed_velocity_gate_without_truth(self) -> None:
        self.assertEqual(self.result["status"], "sealed-structural-no-go")
        self.assertFalse(self.result["decision"]["passed"])
        gnss = self.result["candidate_probe"]["gnss_first"]
        self.assertEqual(gnss["epochs"], 1325)
        self.assertEqual(gnss["velocity_states_exported"], 1325)
        self.assertEqual(gnss["velocity_nonfinite_count"], 0)
        self.assertEqual(gnss["velocity_over_70_mps_count"], 1181)
        self.assertEqual(gnss["positions_clocks_copied"], 0)
        self.assertEqual(gnss["original_raw_seed_position_invalid_count"], 0)
        self.assertEqual(self.result["truth_accounting"]["truth_open_count"], 0)
        self.assertFalse(self.result["truth_accounting"]["mat_read_or_generated"])
        self.assertFalse(self.result["native_policy"]["production_default_changed"])


if __name__ == "__main__":
    unittest.main()
