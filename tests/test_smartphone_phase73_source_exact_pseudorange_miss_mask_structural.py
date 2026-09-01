"""Focused contract tests for the Phase73 truth-free structural runner."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase73_source_exact_pseudorange_miss_mask_structural.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase73_source_exact_pseudorange_miss_mask_structural_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "9d97f1a97bb8559f5f0ede55fcecd109c590b8afb2cf10938b2884d8ffd6d1a0"

_SPEC = importlib.util.spec_from_file_location("phase73_structural_runner", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase73StructuralRunnerTests(unittest.TestCase):
    def test_freeze_pin_and_read_contract(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase73-structural-reads")
        self.assertEqual(freeze["matrix"]["truth_reads"], 0)
        self.assertEqual(freeze["matrix"]["native_solver_invocations"], 12)
        self.assertEqual(freeze["candidate"]["flag"], "--native-base-pseudorange-source-miss-mask")

    def test_artifact_report_keeps_summary_metadata_and_payload_separate(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase73-fixture-") as directory:
            submission = Path(directory) / "submission.csv"
            summary = Path(directory) / "summary.json"
            submission.write_text("fixture", encoding="utf-8")
            summary.write_text("fixture", encoding="utf-8")
            payload = {"sha256": "payload-value", "native": {"enabled": True}}

            def fake_digest(path: Path) -> str:
                return "submission-digest" if path == submission else "summary-digest"

            with mock.patch.object(RUNNER, "read_prediction", return_value=[(1, 1.0, 2.0)]), \
                    mock.patch.object(RUNNER, "speed_report", return_value={"finite": True, "over_70_mps_count": 0}), \
                    mock.patch.object(RUNNER, "validate_summary", return_value={"summary": payload}), \
                    mock.patch.object(RUNNER, "sha256", side_effect=fake_digest):
                report = RUNNER.artifact_report(submission, summary, "fixture/route", False)

            self.assertEqual(report["submission_artifact"]["sha256"], "submission-digest")
            self.assertEqual(report["summary_artifact"]["sha256"], "summary-digest")
            self.assertEqual(report["summary_payload"]["sha256"], "payload-value")
            self.assertNotEqual(report["summary_artifact"]["sha256"], report["summary_payload"]["sha256"])
            self.assertNotIn("summary", report)

    def test_runner_source_forbids_old_collision_and_has_failure_capture(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn('"summary_artifact"', source)
        self.assertIn('"summary_payload"', source)
        self.assertNotIn("**diagnostics", source)
        self.assertIn("except Exception as exc", source)
        self.assertIn("native_solver_invocations_started", source)
        self.assertIn("phase72_partial_output_reused", source)

    def test_candidate_command_has_only_frozen_phase73_opt_in(self) -> None:
        command = RUNNER.native_command(RUNNER.ROUTES[0], Path("output/fixture"), True)
        self.assertIn("--native-base-pseudorange-source-miss-mask", command)
        self.assertIn("--native-base-pseudorange-compensation", command)
        self.assertNotIn("--native-base-pseudorange-preserve-additional-frequency-bands", command)
        self.assertIn("--native-base-rinex-sha256", command)

    def test_no_truth_or_solver_process_is_in_runner_contract(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertFalse(freeze["accuracy"]["authorized"])
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertIn("truth_reads", source)
        self.assertIn("mat_reads_or_generated", source)
        self.assertIn("archive", source)


if __name__ == "__main__":
    unittest.main()
