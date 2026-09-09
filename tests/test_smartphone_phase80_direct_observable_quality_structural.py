"""Focused contract tests for the Phase80 candidate-only runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase80_direct_observable_quality_structural.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase80_source_exact_direct_p_quality_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "9ebdde21fe7fd955029243d3bdbba3a9f9d3832388d67480760394177c8f685e"
_SPEC = importlib.util.spec_from_file_location("phase80_structural_runner", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase80StructuralRunnerTests(unittest.TestCase):
    def test_freeze_is_sealed_candidate_only(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase80-raw-read")
        self.assertEqual(freeze["structural_matrix"]["candidate_runs_total"], 8)
        self.assertEqual(freeze["structural_matrix"]["native_solver_invocations"], 8)
        self.assertEqual(freeze["truth_free_structural_contract"]["truth_reads"], 0) if "truth_reads" in freeze["truth_free_structural_contract"] else self.assertEqual(freeze["structural_matrix"]["truth_reads"], 0)

    def test_command_is_phase78_recipe_plus_direct_quality_with_phase78_band_policy(self) -> None:
        route = RUNNER.ROUTES[0]
        command = RUNNER.native_command(route, Path("output/fixture/candidate/run1"))
        for flag in (*RUNNER.P77.BASE_FLAGS, RUNNER.P77.SIGNAL_BIAS_FLAG, RUNNER.P77.BASE_COMP_FLAG, RUNNER.P77.BASE_RINEX_FLAG, RUNNER.P77.BASE_SHA_FLAG, RUNNER.P77.MISS_MASK_FLAG, RUNNER.DIRECT_FLAG):
            self.assertIn(flag, command)
        self.assertNotIn("--native-upstream-quality", command)
        self.assertNotIn("--native-base-pseudorange-preserve-additional-frequency-bands", command)
        self.assertEqual(command.count(RUNNER.DIRECT_FLAG), 1)

    def test_direct_quality_validation_checks_environment_hubers_counts_and_no_pdc(self) -> None:
        route = RUNNER.ROUTES[0]
        direct_config = {"use_upstream_observable_quality": True, "upstream_snr_percentile": 85.0, "upstream_min_snr_dbhz": 20.0, "upstream_min_elevation_deg": 5.0, "upstream_max_adjacent_gap_s": 1.5, "pseudorange_huber_threshold_sigma": 0.2, "undifferenced_doppler_huber_threshold_sigma": 0.8, "tdcp_sigma_m_unchanged": 0.03, "pseudorange_sigma_contract": "snr_scale*signal_type_factor", "doppler_sigma_contract": "snr_scale/12", "adjacent_mask_contract": "applyAdjacentMasks Pmask_dDP/Lmask_dDL unchanged", "pseudorange_residual_screen": "Pmask_res L1=20m/L5=15m", "doppler_residual_screen": "Dmask_res 3m/s; non-initializer"}
        counts = {key: 2 for key in ("pseudorange_candidates", "pseudorange_factors", "doppler_candidates", "doppler_factors", "doppler_graph_factors", "pseudorange_residual_rejections", "doppler_residual_rejections")}
        summary = {"native_source_direct_observable_quality_enabled": True, "native_upstream_quality": False, "native_pdc_state_bridge": False, "native_pdc_imu_tdcp_no_bridge": True, "native_source_direct_observable_quality": {"enabled": True, "direct_no_pdc": True, "pdc_bridge": False, "native_pdc_state_bridge": False, "environment": "Highway", "p_and_d_huber": {"pseudorange_sigma": 0.2, "doppler_sigma": 0.8}, "config": direct_config, "counts": counts}, "upstream_observable_quality": {"enabled": True, "snr_percentile": 85.0, "snr_denominator_db": 20.0, "min_snr_dbhz": 20.0, "min_elevation_deg": 5.0, "max_adjacent_gap_s": 1.5, "tdcp_sigma_m_unchanged": 0.03, "pseudorange_sigma_contract": "snr_scale*signal_type_factor", "doppler_sigma_contract": "snr_scale/12", **counts}}
        self.assertEqual(RUNNER._validate_direct(summary, route)["counts"], counts)
        bad = json.loads(json.dumps(summary))
        bad["native_source_direct_observable_quality"]["pdc_bridge"] = True
        with self.assertRaises(RUNNER.Phase80StructuralError):
            RUNNER._validate_direct(bad, route)

    def test_runner_keeps_artifact_metadata_separate_and_has_no_controls_or_phase73_identity(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertIn("native_solver_invocations_started", source)
        self.assertIn("summary_payload", source)
        self.assertNotIn("phase73_result", source)
        self.assertNotIn("phase73_miss_mask_identity", source)
        self.assertNotIn("phase43_control", source)
        with tempfile.TemporaryDirectory(prefix="phase80-fixture-") as directory:
            output = Path(directory)
            (output / "submission.csv").write_text("fixture", encoding="utf-8")
            (output / "summary.json").write_text("fixture", encoding="utf-8")
            self.assertNotEqual(RUNNER.relative(output / "submission.csv"), RUNNER.relative(output / "summary.json"))


if __name__ == "__main__":
    unittest.main()
