#!/usr/bin/env python3
"""Focused truth-free contract tests for the Phase40 direct WLS handoff."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
BENCHMARKS = ROOT / "apps" / "commands" / "benchmarks"
SOURCE = ROOT / "apps" / "native" / "gnss_fgo_imu_no_base.cpp"
FREEZE = ROOT / "docs" / "use_cases" / "records" / "smartphone_r5_phase40_direct_doppler_wls_handoff_freeze_v1.json"
sys.path.insert(0, str(BENCHMARKS))

import gnss_smartphone_phase40_direct_doppler_wls_handoff as phase40  # noqa: E402


class SmartphonePhase40DirectDopplerWlsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        cls.source = SOURCE.read_text(encoding="utf-8")

    def test_freeze_routes_gates_and_truth_policy_are_predeclared(self) -> None:
        freeze = phase40.verify_freeze()
        self.assertEqual(freeze["phase"], 40)
        self.assertEqual(freeze["status"], "frozen-before-implementation-and-solver")
        self.assertEqual(tuple(freeze["objective"]["structural_routes"]), phase40.ROUTES)
        self.assertEqual(freeze["objective"]["candidate_flag"], phase40.CANDIDATE_FLAG)
        gates = freeze["predeclared_numeric_gates"]
        self.assertEqual(gates["velocity_norm_max_mps"], 70.0)
        self.assertEqual(gates["clock_rate_abs_max_mps"], 2000.0)
        self.assertEqual(gates["edge_hold_max_s"], 1.0)
        self.assertFalse(gates["gnss_first"]["attempted"])
        self.assertEqual(gates["gnss_first"]["positions_clocks_copied"], 0)
        self.assertEqual(gates["repeat_runs"], 2)

    def test_candidate_command_is_explicit_raw_only_and_flag_off_is_clean(self) -> None:
        paths = {
            "device_gnss": Path("/tmp/device_gnss.csv"),
            "device_imu": Path("/tmp/device_imu.csv"),
            "broadcast_nav": Path("/tmp/brdc.nav"),
        }
        candidate = phase40.native_command(paths, "route/pixel5", Path("/tmp/run1"), True)
        flag_off = phase40.native_command(paths, "route/pixel5", Path("/tmp/run2"), False)
        self.assertIn(phase40.CANDIDATE_FLAG, candidate)
        self.assertNotIn(phase40.CANDIDATE_FLAG, flag_off)
        joined = " ".join(candidate).lower()
        for token in ("ground_truth", "validation", "holdout", ".mat", "kaggle"):
            self.assertNotIn(token, joined)

    def test_source_bypasses_gnss_first_and_consumes_problem_wls_sequence(self) -> None:
        self.assertIn("if (android_raw && options.native_direct_doppler_wls_handoff)", self.source)
        self.assertIn("validateDirectDopplerWlsHandoff(", self.source)
        self.assertIn("problem.doppler_velocity_wls_estimates", self.source)
        self.assertIn("config.use_doppler_velocity_wls_initialization = true", self.source)
        self.assertIn("direct Doppler WLS candidate IMU initialization failed", self.source)
        self.assertIn("options.native_direct_doppler_wls_handoff", self.source)
        direct_start = self.source.index("if (android_raw && options.native_direct_doppler_wls_handoff)")
        gnss_start = self.source.index("} else if (android_raw) {", direct_start)
        direct_block = self.source[direct_start:gnss_start]
        self.assertNotIn("gnss_first_processor", direct_block)
        self.assertIn("validateDirectDopplerWlsHandoff", direct_block)

    def test_candidate_summary_rejects_partial_wls_coverage(self) -> None:
        summary = {
            "dataset_id": "route/pixel5",
            "truth_used": False,
            "production_default_changed": False,
            "native_direct_doppler_wls_handoff": True,
            "native_pdc_imu_tdcp_no_bridge": True,
            "native_quality_anchor": True,
            "native_pdc_state_bridge": False,
            "gnss_first": {
                "attempted": False,
                "handoff_mode": "direct-doppler-wls",
                "positions_clocks_copied": 0,
                "coverage_epochs": 2,
                "coverage_all_epochs": False,
                "direct_valid_count": 1,
                "propagated_valid_count": 0,
                "rejected_count": 1,
                "valid_count": 1,
                "nonfinite_count": 0,
                "over_70_mps_count": 0,
                "clock_rate_over_2000_mps_count": 0,
                "max_velocity_norm_mps": 10.0,
                "max_clock_rate_abs_mps": 10.0,
                "edge_hold_max_s": 1.0,
                "original_raw_seed_position_count": 2,
                "original_raw_seed_position_invalid_count": 0,
            },
            "graph": {"converged": True, "initial_cost": 2.0, "final_cost": 1.0, "imu_intervals": 1},
            "epochs": {"problem": 2, "output": 2, "pseudorange_factors": 1},
            "tdcp_contract": {"factors_built": 1, "factors_inserted": 1, "nonfinite_residuals": 0},
            "raw_utc_key_contract": {"raw_epoch_keys": 2, "target_epochs": 1, "exact_solution_epochs": 1, "interpolated_epochs": 0, "edge_hold_epochs": 0, "unresolved_epochs": 0},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary), encoding="utf-8")
            with self.assertRaises(phase40.Phase40Error):
                phase40.validate_candidate_summary(path, "route/pixel5", 2)


if __name__ == "__main__":
    unittest.main()
