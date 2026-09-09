"""Focused contract tests for the Phase65 structural runner."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase65_native_base_pseudorange_compensation.py"
SPEC = importlib.util.spec_from_file_location("phase65_base", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Phase65NativeBaseContractTests(unittest.TestCase):
    def test_freeze_and_manifest_pins_are_static(self) -> None:
        self.assertEqual(len(MODULE.FREEZE_SHA256), 64)
        self.assertEqual(MODULE.BINARY_SHA256, "f1a3bb43ad68890017cd6e1b273b3df784b002a73bbf606a0f6c5242937403a2")
        self.assertEqual(MODULE.ROUTES, tuple(MODULE.ROUTES))
        self.assertEqual(len(MODULE.ROUTES), 4)
        self.assertEqual(MODULE.BASE_FLAGS[-1], "--native-fallback-seed-quality-anchor-recovery")
        self.assertEqual(MODULE.CANDIDATE_FLAG, "--native-base-pseudorange-compensation")

    def test_candidate_command_declares_route_pinned_base_digest(self) -> None:
        command = MODULE.native_command(
            MODULE.ROUTES[2],
            Path("output/smartphone-r5/phase65-test/route/candidate/run1"),
            True,
        )
        self.assertIn("--native-base-pseudorange-compensation", command)
        self.assertIn("--native-base-rinex", command)
        self.assertIn("--native-base-rinex-sha256", command)
        self.assertEqual(command[-1], MODULE.BASE_INPUT_HASHES[MODULE.ROUTES[2]]["sha256"])
        self.assertNotIn("ground_truth", " ".join(command))

    def test_candidate_summary_requires_coverage_and_materiality(self) -> None:
        route = MODULE.ROUTES[0]
        summary = {
            "dataset_id": route,
            "truth_used": False,
            "production_default_changed": False,
            "native_quality_anchor": True,
            "native_pdc_imu_tdcp_no_bridge": True,
            "native_pdc_state_bridge": False,
            "graph": {"converged": True},
            "epochs": {"problem": 3, "output": 3, "pseudorange_factors": 10},
            "raw_utc_key_contract": {"unresolved_epochs": 0, "target_epochs": 2, "exact_solution_epochs": 2},
            "tdcp_contract": {"factors_built": 2, "factors_inserted": 2, "nonfinite_residuals": 0},
            "native_base_pseudorange_compensation": {
                "enabled": True,
                "built": True,
                "applied": True,
                "base_member_sha256": MODULE.BASE_INPUT_HASHES[route]["sha256"],
                "base_rinex_sha256": MODULE.BASE_INPUT_HASHES[route]["sha256"],
                "base_rinex_bytes": MODULE.BASE_INPUT_HASHES[route]["bytes"],
                "base_coordinate_xyz_m": MODULE.BASE_INPUT_HASHES[route]["xyz"],
                "observed_interval_s": 1.0,
                "moving_mean_samples": 151,
                "same_satellite_signal_only": True,
                "spp_applied": False,
                "tdcp_applied": False,
                "doppler_applied": False,
                "no_extrapolation_or_endpoint_hold": True,
                "adopted_pseudorange_rows": 100,
                "adopted_rows_corrected": 100,
                "finite_correction_fraction": 1.0,
                "correction_abs_p50_m": 0.2,
                "correction_abs_p95_m": 0.5,
            },
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary), encoding="utf-8")
            result = MODULE.validate_summary(path, route, True)
        self.assertTrue(result["base"]["applied"])

    def test_flag_off_rejects_candidate_telemetry(self) -> None:
        route = MODULE.ROUTES[0]
        summary = {
            "dataset_id": route,
            "truth_used": False,
            "production_default_changed": False,
            "native_quality_anchor": True,
            "native_pdc_imu_tdcp_no_bridge": True,
            "native_pdc_state_bridge": False,
            "graph": {"converged": True},
            "epochs": {"problem": 3, "output": 3, "pseudorange_factors": 10},
            "raw_utc_key_contract": {"unresolved_epochs": 0, "target_epochs": 2, "exact_solution_epochs": 2},
            "tdcp_contract": {"factors_built": 2, "factors_inserted": 2, "nonfinite_residuals": 0},
            "native_base_pseudorange_compensation": {},
        }
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "summary.json"
            path.write_text(json.dumps(summary), encoding="utf-8")
            with self.assertRaises(MODULE.Phase65Error):
                MODULE.validate_summary(path, route, False)


if __name__ == "__main__":
    unittest.main()
