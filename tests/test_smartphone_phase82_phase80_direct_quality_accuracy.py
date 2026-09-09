"""Focused tests for the Phase82 artifact-only accuracy scorer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase82_phase80_direct_quality_accuracy.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase82_phase80_direct_quality_accuracy_freeze_v1.json"
PHASE79_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase79_phase78_signal_bias_accuracy_freeze_v1.json"
PHASE81_FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "33bbfc4051af20cd3f39fbf8620ea4d277c4acc5dbe5d8d1621452a657eebf6b"

_SPEC = importlib.util.spec_from_file_location("phase82_direct_quality_accuracy", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase82DirectQualityAccuracyTests(unittest.TestCase):
    def test_freeze_hash_and_pretruth_contract(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["phase"], 82)
        self.assertEqual(freeze["status"], "frozen-before-phase82-truth-read")
        self.assertEqual(freeze["read_policy"]["truth_reads_before_freeze"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_after_manifest"], 4)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)
        self.assertFalse(freeze["read_policy"]["native_or_solver_subprocess"])
        self.assertFalse(freeze["read_policy"]["post_truth_tuning"])
        self.assertTrue(freeze["accuracy_gates"]["declared_before_truth"])

    def test_verify_freeze_and_all_phase80_run1_artifact_pins(self) -> None:
        freeze = RUNNER.verify_freeze()
        self.assertEqual(tuple(freeze["cohort"]["route_order"]), RUNNER.ROUTES)
        pins = freeze["artifact_sources"]["phase80_candidate_run1"]["routes"]
        self.assertEqual(set(pins), set(RUNNER.ROUTES))
        for route in RUNNER.ROUTES:
            pin = pins[route]
            self.assertEqual(len(pin["submission_sha256"]), 64)
            self.assertEqual(len(pin["summary_sha256"]), 64)
            self.assertGreater(pin["submission_bytes"], 0)
            self.assertGreater(pin["summary_bytes"], 0)
            self.assertEqual(Path(pin["submission_path"]).parts[-2], "run1")
            self.assertEqual(Path(pin["summary_path"]).parts[-2], "run1")

    def test_phase81_go_and_phase79_metadata_are_pinned(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        phase79 = json.loads(PHASE79_FREEZE.read_text(encoding="utf-8"))
        phase81 = json.loads(PHASE81_FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["cohort"]["truths"], phase79["cohort"]["truths"])
        self.assertEqual(freeze["artifact_sources"]["phase43_control"], phase79["artifact_sources"]["phase43_control"])
        self.assertEqual(freeze["artifact_sources"]["phase78_candidate_run1"], phase79["artifact_sources"]["phase78_candidate_run1"])
        self.assertEqual(freeze["authority"]["phase81_reclassification_result"]["status"], "go-phase81-phase80-sealed-artifact-reclassification")
        for route in RUNNER.ROUTES:
            current = freeze["artifact_sources"]["phase80_candidate_run1"]["routes"][route]
            sealed = phase81["candidate_artifact_pins"][route]["candidate_run1"]
            self.assertEqual(current["submission_path"], sealed["submission"]["path"])
            self.assertEqual(current["submission_sha256"], sealed["submission"]["sha256"])
            self.assertEqual(current["submission_bytes"], sealed["submission"]["bytes"])
            self.assertEqual(current["summary_path"], sealed["summary"]["path"])
            self.assertEqual(current["summary_sha256"], sealed["summary"]["sha256"])
            self.assertEqual(current["summary_bytes"], sealed["summary"]["bytes"])

    def test_metric_and_strict_gates_are_fixed(self) -> None:
        freeze = RUNNER.verify_freeze()
        phase79 = json.loads(PHASE79_FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["metric_contract"], phase79["metric_contract"])
        gates = freeze["accuracy_gates"]
        self.assertEqual(gates["candidate_improves_each_route_by_at_least_m"], 0.05)
        self.assertEqual(gates["candidate_macro_improvement_at_least_m"], 0.1)
        self.assertEqual(gates["candidate_mtv_h_improvement_at_least_m"], 0.1)
        self.assertEqual(gates["candidate_prediction_domain_coverage"], 1.0)
        self.assertEqual(gates["candidate_macro_score_max_m"], 2.0)
        self.assertEqual(gates["candidate_route_score_max_m"], 3.0)
        self.assertEqual(gates["candidate_mtv_h_p95_max_m"], 5.0)
        self.assertEqual(gates["candidate_over_70_mps_count"], 0)
        self.assertEqual(gates["candidate_macro_score_strict_max_m"], 0.782)
        self.assertEqual(gates["candidate_macro_improvement_vs_phase78_min_m"], 0.0)
        self.assertTrue(gates["candidate_macro_improvement_vs_phase78_strictly_positive"])

    def test_corrected_dictreader_and_haversine_contract(self) -> None:
        route = RUNNER.ROUTES[0]
        payload = (
            b"quality,phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + f"ok,{route},100,37.0,-122.0\n".encode()
        )
        self.assertEqual(RUNNER.P76._parse_truth_dictreader(payload, route), {100: (37.0, -122.0)})
        no_phone = b"UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,quality\n100,37.0,-122.0,ok\n"
        self.assertEqual(RUNNER.P76._parse_truth_dictreader(no_phone, route), {100: (37.0, -122.0)})
        ordered = [(100, 37.0, -122.0), (200, 37.0, -122.0)]
        score = RUNNER.P76._score_prediction({100: (37.0, -122.0)}, {100: (37.0, -122.0), 200: (37.0, -122.0)}, [route, 200], route, ordered)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertEqual(score["missing_truth_rows"], 1)
        self.assertEqual(score["score_m"], 0.0)
        self.assertEqual(score["over_70_mps_count"], 0)

    def test_runner_has_no_native_or_forbidden_input_access(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        self.assertNotIn("device_gnss.csv", source)
        self.assertNotIn("device_imu.csv", source)
        self.assertNotIn("brdc.nav", source)
        self.assertIn("phase80_candidate_artifact_reads", source)
        self.assertIn("phase78_candidate_artifact_reads", source)
        self.assertIn("truth_reads_after_manifest", source)


if __name__ == "__main__":
    unittest.main()
