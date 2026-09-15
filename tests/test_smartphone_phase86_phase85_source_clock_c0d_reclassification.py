"""Focused sealed-artifact contract tests for the Phase86 reclassification."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

from frozen_contract import require_frozen


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase86_phase85_source_clock_c0d_reclassification.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase86_phase85_source_clock_c0d_reclassification_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase86_phase85_source_clock_c0d_reclassification_manifest_v1.json"
EXPECTED_FREEZE_SHA256 = "9b13d6a5118c734521e69d350c9478f0844c5bbf9047c5e057bf4f5a427ace46"

_SPEC = importlib.util.spec_from_file_location("phase86_reclassification", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase86ReclassificationTests(unittest.TestCase):
    def test_freeze_and_phase86_manifest_are_sealed(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = require_frozen(
            "Phase86 freeze",
            RUNNER.Phase86Error,
            RUNNER.verify_freeze,
        )
        self.assertEqual(freeze["phase"], 86)
        self.assertEqual(freeze["status"], "frozen-before-phase86-sealed-artifact-read")
        self.assertTrue(freeze["scope"]["sealed_phase85_artifacts_only"])
        self.assertEqual(freeze["scope"]["native_reruns"], 0)
        self.assertEqual(freeze["scope"]["truth_reads"], 0)
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        self.assertEqual(manifest["status"], "frozen-before-phase86-sealed-artifact-read")
        self.assertEqual(manifest["authority"]["phase85_v2_result"]["bytes"], 641315)
        self.assertEqual(manifest["authority"]["phase85_v2_output_manifest"]["bytes"], 921)

    def test_freeze_pins_all_eight_candidate_pairs(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(tuple(freeze["candidate_artifact_pins"]), RUNNER.ROUTES)
        for route in RUNNER.ROUTES:
            self.assertEqual(set(freeze["candidate_artifact_pins"][route]), {"candidate_run1", "candidate_run2"})
            for run in ("candidate_run1", "candidate_run2"):
                for kind in ("submission", "summary"):
                    pin = freeze["candidate_artifact_pins"][route][run][kind]
                    self.assertEqual(len(pin["sha256"]), 64)
                    self.assertGreater(pin["bytes"], 0)

    def test_summary_active_solve_gate_observations_are_the_formal_no_go(self) -> None:
        route = RUNNER.ROUTES[0]
        summary_path = ROOT / json.loads(FREEZE.read_text(encoding="utf-8"))["candidate_artifact_pins"][route]["candidate_run1"]["summary"]["path"]
        diagnostics = RUNNER.validate_summary(summary_path, route, RUNNER.DOMAIN_ROWS[route])
        graph = diagnostics["graph"]
        self.assertTrue(diagnostics["graph"]["converged"])
        self.assertEqual(graph["iterations"], 0)
        self.assertEqual(graph["initial_cost"], graph["final_cost"])
        self.assertTrue(graph["initial_cost"] == graph["final_cost"])

    def test_submission_parser_and_speed_gate_are_truth_free(self) -> None:
        route = "fixture/route"
        with tempfile.TemporaryDirectory(prefix="phase86-fixture-") as directory:
            path = Path(directory) / "submission.csv"
            path.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{route},1000,37.0,-122.0\n"
                f"{route},2000,37.000001,-122.000001\n",
                encoding="utf-8",
            )
            rows = RUNNER.read_prediction(path, route)
            speed = RUNNER.speed_report(rows)
            self.assertEqual(len(rows), 2)
            self.assertTrue(speed["finite"])
            self.assertEqual(speed["over_70_mps_count"], 0)

    def test_nonempty_output_root_is_refused_without_mutation(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase86-guard-") as directory:
            output = Path(directory) / "out"
            output.mkdir()
            sentinel = output / "sentinel"
            sentinel.write_text("must remain", encoding="utf-8")
            with self.assertRaises(RUNNER.Phase86Error):
                RUNNER.run_reclassification(output)
            self.assertEqual(sentinel.read_text(encoding="utf-8"), "must remain")

    def test_runner_has_no_native_process_or_accuracy_input(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("subprocess", source)
        self.assertNotIn("phase82_result", source)
        self.assertIn("native_reruns", source)
        self.assertIn("candidate_summary_reads", source)
        self.assertIn("graph_iterations_at_least_one", source)
        self.assertIn("final_cost_strictly_less_than_initial_cost", source)


if __name__ == "__main__":
    unittest.main()
