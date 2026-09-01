"""Focused tests for the Phase79 artifact-only accuracy scorer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase79_phase78_signal_bias_accuracy.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_manifest_v1.json"
EXPECTED_FREEZE_SHA256 = "f57b064405cc8e591f03bd1f76c208b364e8fcc4a06c59106a117911d992b166"

_SPEC = importlib.util.spec_from_file_location("phase79_signal_bias_accuracy", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase79SignalBiasAccuracyTests(unittest.TestCase):
    def test_freeze_hash_and_pretruth_contract(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase79-truth-read")
        self.assertEqual(freeze["read_policy"]["truth_reads_before_freeze"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)
        self.assertFalse(freeze["read_policy"]["native_or_solver_subprocess"])
        self.assertTrue(freeze["accuracy_gates"]["declared_before_truth"])

    def test_verify_freeze_and_artifact_route_pins(self) -> None:
        freeze = RUNNER.verify_freeze()
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), RUNNER.ROUTES)
        for source in ("phase78_candidate_run1", "phase43_control", "phase73_no_bias"):
            self.assertEqual(set(freeze["artifact_sources"][source]["routes"]), set(RUNNER.ROUTES))
            for route in RUNNER.ROUTES:
                pin = freeze["artifact_sources"][source]["routes"][route]
                self.assertEqual(len(pin["submission_sha256"]), 64)
                self.assertEqual(len(pin["summary_sha256"]), 64)

    def test_manifest_seals_evaluator_before_truth(self) -> None:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["freeze"]["sha256"], EXPECTED_FREEZE_SHA256)
        self.assertEqual(manifest["evaluator"]["sha256"], hashlib.sha256(RUNNER_PATH.read_bytes()).hexdigest())
        self.assertEqual(manifest["focused_tests"]["sha256"], hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        self.assertEqual(manifest["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(manifest["read_policy"]["truth_reads_after_manifest"], 4)

    def test_corrected_dictreader_accepts_optional_columns_and_phone(self) -> None:
        route = RUNNER.ROUTES[0]
        payload = (
            b"quality,phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"ok,{route},100,37.0,-122.0\n".encode()
        )
        self.assertEqual(RUNNER.P76._parse_truth_dictreader(payload, route), {100: (37.0, -122.0)})
        no_phone = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,quality\n100,37.0,-122.0,ok\n"
        self.assertEqual(RUNNER.P76._parse_truth_dictreader(no_phone, route), {100: (37.0, -122.0)})

    def test_corrected_dictreader_rejects_wrong_phone_and_unnamed_cells(self) -> None:
        route = RUNNER.ROUTES[0]
        wrong_phone = b"phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\nother,100,37,-122\n"
        with self.assertRaises(RUNNER.P76.Phase76RecoveryError):
            RUNNER.P76._parse_truth_dictreader(wrong_phone, route)
        unnamed = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n100,37,-122,extra\n"
        with self.assertRaises(RUNNER.P76.Phase76RecoveryError):
            RUNNER.P76._parse_truth_dictreader(unnamed, route)

    def test_metric_reuses_exact_haversine_score_and_warmup_contract(self) -> None:
        route = RUNNER.ROUTES[0]
        ordered = [(100, 37.0, -122.0), (200, 37.0, -122.0)]
        prediction = {100: (37.0, -122.0)}
        truth = {100: (37.0, -122.0), 200: (37.0, -122.0)}
        score = RUNNER.P76._score_prediction(prediction, truth, [route, 200], route, ordered)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertEqual(score["missing_truth_rows"], 1)
        self.assertEqual(score["score_m"], 0.0)
        self.assertEqual(score["over_70_mps_count"], 0)

    def test_runner_has_no_native_or_forbidden_input_access(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        self.assertNotIn("native_or_solver_subprocess()", source)
        self.assertIn("truth_reads_per_route", source)
        self.assertIn("phase73_no_bias", source)
        self.assertIn("phase78_candidate_artifact_reads", source)


if __name__ == "__main__":
    unittest.main()
