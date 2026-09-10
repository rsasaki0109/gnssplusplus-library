"""Focused contract tests for the Phase77 truth-free structural runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase77_phase73_signal_bias_composition_structural.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase77_phase73_signal_bias_composition_structural_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "4f7bd6136180bb86a81031eaff38689db1b44451fb418d21cf6d796d27fb4e95"

_SPEC = importlib.util.spec_from_file_location("phase77_structural_runner", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase77StructuralRunnerTests(unittest.TestCase):
    def test_freeze_pin_and_identity_not_historical_retention_threshold(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase77-raw-read")
        gates = freeze["truth_free_structural_gates"]
        self.assertTrue(gates["phase73_miss_mask_telemetry_identity"])
        self.assertIsInstance(gates["base_matched_adopted_factor_fraction"], str)
        self.assertIsInstance(gates["base_finite_correction_fraction"], str)
        self.assertNotIn("base_coverage_below_0_80", gates)
        self.assertNotIn("base_finite_correction_below_0_99", gates)
        self.assertEqual(freeze["matrix"]["truth_reads"], 0)
        self.assertEqual(freeze["matrix"]["native_invocations"], 12)

    def test_commands_keep_phase73_control_and_add_bias_only_to_candidate(self) -> None:
        route = RUNNER.ROUTES[0]
        control = RUNNER.native_command(route, Path("output/fixture/control"), False)
        candidate = RUNNER.native_command(route, Path("output/fixture/candidate"), True)
        self.assertIn("--native-base-pseudorange-compensation", control)
        self.assertIn("--native-base-pseudorange-source-miss-mask", control)
        self.assertNotIn("--native-signal-bias-states", control)
        self.assertIn("--native-signal-bias-states", candidate)
        self.assertNotIn("--native-base-pseudorange-preserve-additional-frequency-bands", candidate)

    def test_phase73_telemetry_identity_is_exact(self) -> None:
        route = RUNNER.ROUTES[0]
        expected = RUNNER._expected_miss_telemetry(route)
        passed = RUNNER._miss_identity(route, dict(expected))
        self.assertTrue(passed["passed"])
        altered = dict(expected)
        altered["retained_finite_pc_pseudorange_rows"] += 1
        failed = RUNNER._miss_identity(route, altered)
        self.assertFalse(failed["passed"])
        self.assertIn("retained_finite_pc_pseudorange_rows", failed["mismatches"])

    def test_signal_bias_validation_requires_finite_material_states(self) -> None:
        summary = {
            "native_signal_bias_states": True,
            "epochs": {"receiver_signal_bias_factors": 10, "receiver_signal_bias_states": 2},
            "receiver_signal_bias_estimates_m": {"1": -3.0, "2": 4.5},
        }
        signal = RUNNER._validate_signal_bias(summary, "fixture/route")
        self.assertEqual(signal["states"], 2)
        self.assertEqual(signal["factors"], 10)
        bad = dict(summary)
        bad["receiver_signal_bias_estimates_m"] = {"1": float("nan"), "2": 4.5}
        with self.assertRaises(RUNNER.Phase77StructuralError):
            RUNNER._validate_signal_bias(bad, "fixture/route")

    def test_artifact_metadata_and_summary_payload_do_not_collide(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase77-fixture-") as directory:
            submission = Path(directory) / "submission.csv"
            summary = Path(directory) / "summary.json"
            submission.write_text("fixture", encoding="utf-8")
            summary.write_text("fixture", encoding="utf-8")
            payload = {
                "summary_marker": "payload",
                "epochs": {"pseudorange_factors": 10, "output": 4},
                "tdcp_contract": {"factors_built": 2, "factors_inserted": 2},
                "graph": {"imu_intervals": 3},
                "upstream_observable_quality": {"doppler_graph_factors": 5},
            }

            def fake_digest(path: Path) -> str:
                return "submission-digest" if path == submission else "summary-digest"

            with mock.patch.object(RUNNER, "read_prediction", return_value=[(1, 1.0, 2.0)]), \
                    mock.patch.object(RUNNER, "speed_report", return_value={"finite": True, "over_70_mps_count": 0}), \
                    mock.patch.object(RUNNER, "validate_summary", return_value={"summary": payload, "base": {}, "source_miss_mask": {}, "phase73_miss_mask_identity": {"passed": True}, "signal_bias": {"states": 1, "factors": 1, "estimates_m": {"x": 0.0}}}), \
                    mock.patch.object(RUNNER, "sha256", side_effect=fake_digest):
                report = RUNNER.artifact_report(submission, summary, "fixture/route", True)
            self.assertEqual(report["submission_artifact"]["sha256"], "submission-digest")
            self.assertEqual(report["summary_artifact"]["sha256"], "summary-digest")
            self.assertEqual(report["summary_payload"]["summary_marker"], "payload")
            self.assertNotIn("summary", report)

    def test_runner_source_has_fail_closed_accounting_and_no_truth_stage(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertIn("truth_reads", source)
        self.assertIn("native_solver_invocations_started", source)
        self.assertIn("phase73_output_reused", source)
        self.assertIn("summary_artifact", source)
        self.assertIn("summary_payload", source)
        self.assertNotIn("**diagnostics", source)


if __name__ == "__main__":
    unittest.main()
