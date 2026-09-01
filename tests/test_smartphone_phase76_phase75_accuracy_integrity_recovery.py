"""Focused tests for the Phase76 scorer-only accuracy recovery."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase76_phase75_accuracy_integrity_recovery.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase76_phase75_accuracy_integrity_recovery_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "6674773e4d988644f205b52bdae5228dcaa0160ac84a58f63d5192035ee121c8"
ROUTE = "2021-03-16-18-59-us-ca-mtv-a/pixel5"

_SPEC = importlib.util.spec_from_file_location("phase76_accuracy_recovery", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase76AccuracyRecoveryTests(unittest.TestCase):
    def test_freeze_pin_and_read_contract(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase76-truth-read")
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)
        self.assertIn("sealed_phase73_artifacts.routes[route].control", freeze["sealed_phase73_artifact_source"]["route_control_identity"])

    def test_dictreader_accepts_named_extra_column_and_optional_phone(self) -> None:
        payload = (
            b"quality,phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"ok,{ROUTE},100,37.0,-122.0\n".encode()
        )
        self.assertEqual(RUNNER._parse_truth_dictreader(payload, ROUTE), {100: (37.0, -122.0)})

    def test_dictreader_assigns_route_without_phone(self) -> None:
        payload = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,quality\n100,37.0,-122.0,ok\n"
        self.assertEqual(RUNNER._parse_truth_dictreader(payload, ROUTE), {100: (37.0, -122.0)})

    def test_dictreader_rejects_wrong_phone_and_unnamed_cells(self) -> None:
        wrong_phone = b"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\nother,100,37,-122\n"
        with self.assertRaises(RUNNER.Phase76RecoveryError):
            RUNNER._parse_truth_dictreader(wrong_phone, ROUTE)
        unnamed = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n100,37,-122,unexpected\n"
        with self.assertRaises(RUNNER.Phase76RecoveryError):
            RUNNER._parse_truth_dictreader(unnamed, ROUTE)

    def test_score_route_executes_direct_phase74_control_identity(self) -> None:
        """Exercise score_route through its direct nested-control identity line."""
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            candidate_submission_path = root / "candidate.csv"
            candidate_summary_path = root / "candidate.json"
            control_submission_path = root / "control.csv"
            control_summary_path = root / "control.json"
            truth_path = root / "truth.csv"
            candidate_submission = b"candidate"
            candidate_summary = b"candidate-summary"
            control_submission = b"control"
            control_summary = b"control-summary"
            truth_payload = b"truth"
            for path, payload in (
                (candidate_submission_path, candidate_submission),
                (candidate_summary_path, candidate_summary),
                (control_submission_path, control_submission),
                (control_summary_path, control_summary),
                (truth_path, truth_payload),
            ):
                path.write_bytes(payload)

            candidate_pin = {
                "submission_path": str(candidate_submission_path),
                "submission_sha256": hashlib.sha256(candidate_submission).hexdigest(),
                "submission_bytes": len(candidate_submission),
                "summary_path": str(candidate_summary_path),
                "summary_sha256": hashlib.sha256(candidate_summary).hexdigest(),
            }
            control_pin = {
                "submission_path": str(control_submission_path),
                "submission_sha256": hashlib.sha256(control_submission).hexdigest(),
                "submission_bytes": len(control_submission),
                "summary_path": str(control_summary_path),
                "summary_sha256": hashlib.sha256(control_summary).hexdigest(),
            }
            freeze74 = {"sealed_phase73_artifacts": {"routes": {ROUTE: {"candidate": candidate_pin, "control": control_pin}}}}
            freeze = {"cohort": {"truths": {ROUTE: {"path": str(truth_path), "sha256": hashlib.sha256(truth_payload).hexdigest(), "bytes": len(truth_payload), "rows": 1, "expected_missing_truth_rows": 0, "expected_missing_key": None}}}}
            telemetry = {
                "original_adopted_pseudorange_rows": 1,
                "retained_finite_pc_pseudorange_rows": 1,
                "dropped_missing_exact_stream_rows": 0,
                "dropped_out_of_domain_rows": 0,
                "dropped_nonfinite_correction_rows": 0,
                "retained_over_original_fraction": 1.0,
                "correction_abs_p50_m": 1.0,
                "correction_abs_p95_m": 1.0,
                "correction_abs_max_m": 1.0,
            }
            fake_score = {
                "prediction_domain_coverage": 1.0,
                "finite": True,
                "over_70_mps_count": 0,
                "p95_m": 1.0,
                "score_m": 1.0,
            }

            def fake_parse(payload: bytes, route: str):
                self.assertEqual(route, ROUTE)
                return ([(100, 37.0, -122.0)], {100: (37.0, -122.0)})

            accounting = {"candidate_artifact_reads": 0, "control_artifact_reads": 0, "truth_reads": 0}
            with mock.patch.object(RUNNER.P74, "_parse_submission", side_effect=fake_parse), mock.patch.object(RUNNER.P74, "_validate_summary", return_value={"miss_mask": telemetry}), mock.patch.object(RUNNER, "_parse_truth_dictreader", return_value={100: (37.0, -122.0)}), mock.patch.object(RUNNER, "_score_prediction", return_value=fake_score):
                report = RUNNER.score_route(freeze, freeze74, ROUTE, accounting)

            self.assertTrue(report["control_phase43_identity"])
            self.assertEqual(report["control_identity_source"], "Phase74 freeze sealed_phase73_artifacts.routes[route].control")
            self.assertEqual(accounting, {"candidate_artifact_reads": 2, "control_artifact_reads": 2, "truth_reads": 1})

    def test_no_native_or_solver_process_and_new_output_root(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["output"]["new_output_root"], "output/smartphone-r5/phase76-phase75-accuracy-integrity-recovery-v1")
        self.assertFalse(freeze["read_policy"]["native_or_solver_subprocess"])


if __name__ == "__main__":
    unittest.main()
