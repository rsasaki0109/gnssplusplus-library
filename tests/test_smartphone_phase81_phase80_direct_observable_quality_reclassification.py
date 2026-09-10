"""Focused tests for the Phase81 sealed-artifact reclassification."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase81_phase80_direct_observable_quality_reclassification.py"
FREEZE = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_freeze_v1.json"
MANIFEST = ROOT / "docs/use_cases/records/smartphone_r5_phase81_phase80_direct_observable_quality_reclassification_manifest_v1.json"
EXPECTED_FREEZE_SHA256 = "c648ec5f49f9023239360a70a1d77ee4b546e2b4d6f07cdf076f00694a2aeb6f"

_SPEC = importlib.util.spec_from_file_location("phase81_reclassification", RUNNER_PATH)
if _SPEC is None or _SPEC.loader is None:  # pragma: no cover
    raise RuntimeError(f"failed to load runner: {RUNNER_PATH}")
RUNNER = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(RUNNER)


class Phase81ReclassificationTests(unittest.TestCase):
    def test_freeze_is_sealed_and_zero_native_truth_reads(self) -> None:
        self.assertEqual(hashlib.sha256(FREEZE.read_bytes()).hexdigest(), EXPECTED_FREEZE_SHA256)
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(freeze["status"], "frozen-before-phase81-sealed-artifact-read")
        self.assertTrue(freeze["scope"]["no_native_rerun"])
        self.assertTrue(freeze["scope"]["no_accuracy_truth"])
        self.assertEqual(freeze["fixed_reclassification_gates"]["candidate_prediction_domain_coverage"], 1.0)
        self.assertEqual(freeze["read_accounting_at_freeze"]["native_solver_invocations"], 0)
        self.assertEqual(freeze["read_accounting_at_freeze"]["truth"], 0)

    def test_failure_and_eight_candidate_pairs_are_pinned(self) -> None:
        freeze = json.loads(FREEZE.read_text(encoding="utf-8"))
        self.assertEqual(len(freeze["candidate_artifact_pins"]), 4)
        for route in RUNNER.ROUTES:
            self.assertEqual(set(freeze["candidate_artifact_pins"][route]), {"candidate_run1", "candidate_run2"})
            for run in ("candidate_run1", "candidate_run2"):
                for kind in ("submission", "summary"):
                    pin = freeze["candidate_artifact_pins"][route][run][kind]
                    self.assertEqual(len(pin["sha256"]), 64)
                    self.assertGreater(pin["bytes"], 0)
        failure = freeze["authority"]["phase80_v2_failure"]
        self.assertEqual(failure["bytes"], 600598)
        self.assertEqual(len(failure["sha256"]), 64)

    def test_legacy_diagnostic_has_only_domain_leaf_false(self) -> None:
        failure = json.loads((ROOT / "output/smartphone-r5/phase80-source-exact-direct-observable-quality-structural-v2/phase80_direct_observable_quality_structural_failure.json").read_text(encoding="utf-8"))
        self.assertEqual(failure["errors"], [])
        for route in RUNNER.ROUTES:
            gates = failure["routes"][route]["gates"]
            self.assertFalse(gates["all_route_gates"])
            self.assertFalse(gates["prediction_domain_coverage_exact"])
            self.assertTrue(all(value is True for key, value in gates.items() if key not in ("all_route_gates", "prediction_domain_coverage_exact")))

    def test_rows_plus_warmup_and_speed_parser_are_truth_free(self) -> None:
        route = "fixture/route"
        with tempfile.TemporaryDirectory(prefix="phase81-fixture-") as directory:
            path = Path(directory) / "submission.csv"
            path.write_text("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" f"{route},1000,37.0,-122.0\n" f"{route},2000,37.000001,-122.000001\n", encoding="utf-8")
            rows = RUNNER._read_prediction(path, route)
            speed = RUNNER._speed_report(rows)
            self.assertEqual(len(rows) + 1, 3)
            self.assertTrue(speed["finite"])
            self.assertEqual(speed["over_70_mps_count"], 0)

    def test_runner_has_no_native_subprocess_or_accuracy_input(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8").lower()
        self.assertNotIn("subprocess", source)
        self.assertNotIn("subprocess.run", source)
        self.assertIn("native_solver_invocations", source)
        self.assertIn("candidate_prediction_domain_coverage", source)
        self.assertIn("direct_no_pdc", source)


if __name__ == "__main__":
    unittest.main()
