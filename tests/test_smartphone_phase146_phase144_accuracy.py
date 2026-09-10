"""Synthetic launch-free tests for the Phase146 evaluator wiring."""

from __future__ import annotations

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase146_phase144_accuracy.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load(RUNNER_PATH, "phase146_accuracy_test_runner")


class Phase146AccuracyTests(unittest.TestCase):
    def test_manifest_selects_phase144_not_phase141_or_phase142(self) -> None:
        candidate = RUNNER.verify_phase144_metadata()
        self.assertEqual(candidate[RUNNER.ROUTES[0]]["sha256"], "b765951daa5478600f5da1f0ad0a27bdc5cdf265d681df21d91779abf3163b00")
        self.assertTrue(all("phase144-telemetry-serializer" in item["path"] for item in candidate.values()))
        self.assertNotIn("phase141-telemetry-schema", json.dumps(candidate))

    def test_metric_contract_is_exact_pinned_reference(self) -> None:
        manifest = RUNNER.verify_manifest()
        self.assertEqual(manifest["metric_contract"], RUNNER.metric_contract())

    def test_no_double_offset_and_strict_threshold(self) -> None:
        manifest = RUNNER.verify_manifest()
        self.assertEqual(manifest["candidate_reference"]["pixel5_offset_reapplication"], 0)
        self.assertTrue(RUNNER.strict_macro_gate(0.781999999))
        self.assertFalse(RUNNER.strict_macro_gate(0.782))
        self.assertFalse(RUNNER.strict_macro_gate(float("nan")))

    def test_full_alignment_domain_finite_velocity_gates(self) -> None:
        route = RUNNER.ROUTES[0]
        good = {"prediction_domain_coverage": 1.0, "finite": True, "over_70_mps_count": 0, "score_m": 0.1}
        self.assertTrue(all(RUNNER.route_gates(RUNNER.DOMAIN_ROWS[route], good, route).values()))
        for bad in ({**good, "prediction_domain_coverage": 0.99}, {**good, "finite": False}, {**good, "over_70_mps_count": 1}, {**good, "score_m": float("inf")}):
            self.assertFalse(all(RUNNER.route_gates(RUNNER.DOMAIN_ROWS[route], bad, route).values()))
        self.assertFalse(all(RUNNER.route_gates(RUNNER.DOMAIN_ROWS[route] - 1, good, route).values()))

    def test_duplicate_json_and_unauthorized_materialization_fail_closed(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
            handle.write('{"route": 1, "route": 2}')
            handle.flush()
            with self.assertRaises(RUNNER.Phase146Error):
                RUNNER.read_json(Path(handle.name), "synthetic duplicate")
        with self.assertRaises(RUNNER.Phase146Error):
            RUNNER.materialize("output/smartphone-r5/phase144-telemetry-serializer-structural-v1/x/opaque_solution_output.csv", "opaque_solution_output.csv", "/phase144-telemetry-serializer-structural-v1/", "candidate", False)


if __name__ == "__main__":
    unittest.main()
