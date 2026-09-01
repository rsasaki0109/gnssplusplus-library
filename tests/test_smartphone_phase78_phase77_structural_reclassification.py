"""Focused tests for the Phase78 sealed-artifact reclassification."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase78_phase77_structural_reclassification.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase78_phase77_structural_reclassification_freeze_v1.json"
EXPECTED_FREEZE_SHA256 = "0214783abba6584383085c00824a3c227d7e17fdc041e369fcdd369e48baf897"

_SPEC = importlib.util.spec_from_file_location("phase78_reclassification", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase78ReclassificationTests(unittest.TestCase):
    def test_freeze_pin_is_sealed_and_zero_read(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase78-sealed-artifact-read")
        self.assertTrue(freeze["scope"]["no_native_rerun"])
        self.assertTrue(freeze["scope"]["no_accuracy_truth"])
        self.assertEqual(freeze["fixed_reclassification_gates"]["candidate_prediction_domain_coverage"], 1.0)
        self.assertEqual(freeze["read_accounting_at_freeze"]["native_solver_invocations"], 0)
        self.assertEqual(freeze["read_accounting_at_freeze"]["truth"], 0)

    def test_candidate_artifact_pins_cover_all_routes_and_two_repeats(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        pins = freeze["candidate_artifact_pins"]
        self.assertEqual(tuple(pins), RUNNER.ROUTES)
        for route in RUNNER.ROUTES:
            self.assertEqual(set(pins[route]), {"candidate_run1", "candidate_run2"})
            for run in ("candidate_run1", "candidate_run2"):
                self.assertEqual(len(pins[route][run]["submission_sha256"]), 64)
                self.assertEqual(len(pins[route][run]["summary_sha256"]), 64)
                self.assertEqual(pins[route][run]["submission_sha256"], pins[route]["candidate_run1"]["submission_sha256"])

    def test_prediction_parser_and_speed_gate_are_truth_free(self) -> None:
        route = "fixture/route"
        with tempfile.TemporaryDirectory(prefix="phase78-fixture-") as directory:
            path = Path(directory) / "submission.csv"
            path.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{route},1000,37.0,-122.0\n"
                f"{route},2000,37.000001,-122.000001\n",
                encoding="utf-8",
            )
            rows = RUNNER._read_prediction(path, route)
            speed = RUNNER._speed_report(rows)
            self.assertEqual(len(rows), 2)
            self.assertTrue(speed["finite"])
            self.assertEqual(speed["over_70_mps_count"], 0)

    def test_finite_pc_factor_accounting_rejects_mismatch(self) -> None:
        route = RUNNER.ROUTES[0]
        expected = json.loads(FREEZE.read_text(encoding="utf-8"))["candidate_telemetry_pins"][route]
        self.assertEqual(expected["retained_finite_pc_fraction"], 1.0)
        self.assertEqual(expected["pseudorange_factors"], expected["retained_finite_pc_pseudorange_rows"])
        self.assertEqual(expected["tdcp_factors_built"], expected["tdcp_factors_inserted"])
        self.assertGreaterEqual(expected["receiver_signal_bias_states"], 1)
        self.assertGreaterEqual(expected["receiver_signal_bias_factors"], 1)

    def test_runner_contains_no_native_subprocess_or_raw_input_contract(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("subprocess.run", source)
        self.assertNotIn("native_solver_invocations", source.split("def run_reclassification", 1)[1].split("def main", 1)[0])
        self.assertIn("native_reruns", source)
        self.assertIn("raw_reads", source)
        self.assertIn("truth_reads", source)
        self.assertIn("candidate_artifact_hashes_exact", source)


if __name__ == "__main__":
    unittest.main()
