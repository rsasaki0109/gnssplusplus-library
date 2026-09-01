#!/usr/bin/env python3
"""Truth-free focused tests for the Phase52 integrity-recovery scorer."""

from __future__ import annotations

import ast
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCORER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase52_android_sv_time_uncertainty_integrity_recovery.py"
SOURCE = SCORER_PATH.read_text(encoding="utf-8")
SPEC = importlib.util.spec_from_file_location("phase52_scorer", SCORER_PATH)
assert SPEC is not None and SPEC.loader is not None
phase52 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(phase52)


class Phase52IntegrityRecoveryTest(unittest.TestCase):
    def test_scorer_has_no_solver_or_process_launch_path(self) -> None:
        tree = ast.parse(SOURCE)
        imports = [node for node in ast.walk(tree) if isinstance(node, (ast.Import, ast.ImportFrom))]
        self.assertFalse(any(alias.name == "subprocess" for node in imports for alias in node.names))
        self.assertNotIn("Popen(", SOURCE)
        self.assertNotIn("check_output(", SOURCE)
        self.assertNotIn("gnss_fgo_imu_no_base", SOURCE)

    def test_warmup_missing_first_truth_key_is_allowed(self) -> None:
        route = phase52.ROUTES[0]
        truth = {(route, 1000): (37.0, -122.0), (route, 2000): (37.0, -122.0)}
        rows = [(route, 2000, 37.0, -122.0)]
        domain = phase52.check_domain(rows, truth, route, 1, "synthetic")
        self.assertEqual(domain["prediction_domain_coverage"], 1.0)
        self.assertEqual(domain["truth_row_coverage"], 0.5)
        self.assertEqual(domain["warmup_missing_key"], 1000)

    def test_arbitrary_missing_truth_key_fails_closed(self) -> None:
        route = phase52.ROUTES[0]
        truth = {(route, 1000): (37.0, -122.0), (route, 2000): (37.0, -122.0), (route, 3000): (37.0, -122.0)}
        rows = [(route, 1000, 37.0, -122.0), (route, 3000, 37.0, -122.0)]
        with self.assertRaises(phase52.Phase52Error):
            phase52.check_domain(rows, truth, route, 1, "synthetic")

    def test_frozen_gates_and_route_set_are_unchanged(self) -> None:
        freeze = phase52.verify_freeze()
        self.assertEqual(tuple(freeze["immutable_phase51_outputs"]["route_order"]), phase52.ROUTES)
        self.assertEqual(freeze["domain_contract"]["truth_row_coverage"], "informational only")
        self.assertEqual(freeze["accuracy_gates"]["candidate_improvement_each_route_min_m"], 0.05)
        self.assertEqual(freeze["accuracy_gates"]["macro_improvement_min_m"], 0.10)
        self.assertEqual(freeze["accuracy_gates"]["mtv_h_improvement_min_m"], 0.10)


if __name__ == "__main__":
    unittest.main()
