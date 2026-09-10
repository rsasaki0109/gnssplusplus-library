"""Focused, truth-free tests for the Phase74 accuracy scorer."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase74_phase73_miss_mask_accuracy.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase74_phase73_miss_mask_accuracy_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "8308f15a491d1ef1daae0542efd1d7ac96562738e51e6c90ea04fd9f81aba8d9"

_SPEC = importlib.util.spec_from_file_location("phase74_accuracy_runner", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase74AccuracyRunnerTests(unittest.TestCase):
    def test_freeze_is_unchanged_and_truth_is_not_authorized_before_manifest(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase74-truth-read")
        self.assertEqual(freeze["read_policy"]["truth_reads_before_manifest"], 0)
        self.assertEqual(freeze["read_policy"]["truth_reads_per_route"], 1)
        self.assertFalse(freeze["authority"]["phase51_historical_reference"]["scoring_input"])
        self.assertFalse(freeze["authority"]["phase52_historical_recovery_reference"]["scoring_input"])

    def test_source_exact_metric_percentile_and_warmup_contract(self) -> None:
        self.assertAlmostEqual(RUNNER._percentile([0.0, 10.0], 0.95), 9.5)
        route = "fixture/route"
        truth = {1: (0.0, 0.0), 2: (0.0, 0.0)}
        prediction = {1: (0.0, 0.0)}
        result = RUNNER._score_prediction(
            prediction,
            truth,
            [route, 2],
            route,
            [(1, 0.0, 0.0)],
        )
        self.assertEqual(result["prediction_domain_coverage"], 1.0)
        self.assertEqual(result["truth_row_coverage"], 0.5)
        self.assertEqual(result["missing_truth_keys"], [[route, 2]])
        self.assertEqual(result["score_m"], 0.0)

    def test_read_once_hashes_and_size_checks_one_payload(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase74-fixture-") as directory:
            path = Path(directory) / "artifact.json"
            payload = b"{}\n"
            path.write_bytes(payload)
            self.assertEqual(
                RUNNER._read_once(path, hashlib.sha256(payload).hexdigest(), len(payload), "fixture"),
                payload,
            )

    def test_scorer_has_no_native_or_solver_subprocess_path(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("subprocess.run", source)
        self.assertIn("native_or_solver_subprocess", source)
        self.assertIn("truth_reads", source)
        self.assertIn("post_truth_tuning", source)

    def test_artifact_and_truth_routes_are_frozen(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        routes = freeze["cohort"]["route_order"]
        self.assertEqual(len(routes), 4)
        for route in routes:
            self.assertIn(route, freeze["cohort"]["truths"])
            self.assertIn(route, freeze["sealed_phase73_artifacts"]["routes"])
            self.assertEqual(freeze["cohort"]["truths"][route]["expected_missing_truth_rows"], 1 if route.endswith(("mtv-a/pixel5", "mtv-u/pixel5")) else 0)


if __name__ == "__main__":
    unittest.main()
