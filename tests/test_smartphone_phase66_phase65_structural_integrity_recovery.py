#!/usr/bin/env python3
"""Focused, raw-free tests for the Phase66 nested-hash recovery contract."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase66_phase65_structural_integrity_recovery.py"
MANIFEST_PATH = ROOT / "docs/use_cases/records/smartphone_r5_phase66_phase65_structural_integrity_recovery_manifest_v1.json"


def load_runner():
    spec = importlib.util.spec_from_file_location("phase66_structural_recovery", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("cannot load Phase66 runner")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase66NestedHashSchemaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load_runner()

    def test_fixture_preserves_nested_phase43_schema(self):
        fixture = self.runner.flat_reference_fixture()
        self.assertEqual(set(fixture), {"submission", "summary"})
        self.assertEqual(fixture["submission"]["sha256"], "submission-digest")
        self.assertEqual(fixture["summary"]["sha256"], "summary-digest")
        self.runner.compare_control_reference(fixture, fixture, "fixture")

    def test_nested_mismatch_fails_closed(self):
        fixture = self.runner.flat_reference_fixture()
        mismatch = {**fixture, "summary": {**fixture["summary"], "sha256": "wrong"}}
        with self.assertRaises(self.runner.Phase66Error):
            self.runner.compare_control_reference(fixture, mismatch, "fixture")

    def test_manifest_and_freeze_schema_pins(self):
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        self.assertEqual(manifest["freeze"]["sha256"], self.runner.FREEZE_SHA256)
        self.assertEqual(manifest["evaluator"]["sha256"], "837c0ffce5e386d26cbc792adcfdbde831b7b3256234d89e3cb35ccd5bdabe2f")
        self.assertEqual(manifest["source_commit"], "f8cdf3b")
        self.assertEqual(manifest["routes"], list(self.runner.ROUTES))
        self.assertEqual(manifest["read_accounting"]["truth_reads"], 0)
        self.assertIs(manifest["read_accounting"]["partial_phase65_output_reuse"], False)
        self.assertEqual(manifest["structural_matrix"]["native_solver_invocations"], 12)
        self.assertEqual(manifest["structural_matrix"]["control_runs_per_route"], 1)
        self.assertEqual(manifest["structural_matrix"]["candidate_runs_per_route"], 2)

    def test_base_route_hash_and_bytes_pins(self):
        manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))
        for route in self.runner.ROUTES:
            expected = self.runner.P65.BASE_INPUT_HASHES[route]
            actual = manifest["base_inputs"][route]
            self.assertEqual(actual["sha256"], expected["sha256"])
            self.assertEqual(actual["bytes"], expected["bytes"])

    def test_partial_phase65_output_is_not_a_matrix_input(self):
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertIn("partial_phase65_output_reuse", source)
        self.assertIn("P65_FAILURE", source)
        self.assertNotIn("phase65-native-base-pseudorange-compensation-v1/candidate", source)


if __name__ == "__main__":
    unittest.main()
