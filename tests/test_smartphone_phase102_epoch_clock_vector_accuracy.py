"""Launch-free tests for the Phase102 accuracy contract.

Only synthetic CSV bytes and committed metadata are used.  These tests do
not open raw inputs or official truth, launch the native solver, inspect a
solution from a prior phase, or access MAT/base/Kaggle lanes.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase102_epoch_clock_vector_accuracy.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase102AccuracyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_module(CONTRACT_PATH, "phase102_epoch_clock_vector_accuracy_contract_test")

    def test_freeze_is_phase101_source_parity_and_strict_gate_is_frozen(self):
        freeze = self.contract.verify_freeze()
        self.assertEqual(freeze["phase"], 102)
        self.assertEqual(freeze["candidate"]["id"], self.contract.CANDIDATE_ID)
        self.assertEqual(freeze["candidate"]["routes"], list(self.contract.ROUTES))
        self.assertFalse(freeze["candidate"]["phase101_withheld_solution_reuse"])
        self.assertEqual(freeze["promotion_gates"]["candidate_macro_score_strict_max_m"], 0.782)
        self.assertTrue(freeze["promotion_gates"]["strict_0_782_is_explicit_and_gate"])

    def test_manifest_has_exact_two_fresh_raw_only_commands(self):
        manifest = self.contract.verify_manifest()
        self.assertEqual([item["dataset_id"] for item in manifest["routes"]], list(self.contract.ROUTES))
        self.assertEqual(manifest["matrix"]["native_invocations_planned"], 2)
        self.assertEqual(manifest["matrix"]["truth_reads_in_native"], 0)
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count(self.contract.SELECTOR), 1)
            self.assertEqual(command.count(self.contract.QR_SELECTOR), 1)
            self.assertEqual(command.count(self.contract.HANDOFF_SELECTOR), 1)
            self.assertEqual(command[command.index("--out") + 1].endswith("/solution.csv"), True)
            self.assertFalse(any(flag in command for flag in self.contract.FORBIDDEN_FLAGS))
            self.assertNotIn("truth", " ".join(command).lower())

    def test_synthetic_metric_uses_pinned_phase82_parser(self):
        p82 = self.contract._import_phase82()
        route = self.contract.ROUTES[0]
        submission = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            f"{route},1000,37.0000000,-122.0000000\n"
            f"{route},2000,37.0001000,-122.0000000\n"
        ).encode()
        truth = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,optional\n"
            "1000,37.0000000,-122.0000000,x\n"
            "2000,37.0000000,-122.0000000,x\n"
        ).encode()
        ordered, prediction = p82.P76.P74._parse_submission(submission, route)
        truth_map = p82.P76._parse_truth_dictreader(truth, route)
        score = p82.P76._score_prediction(prediction, truth_map, None, route, ordered)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertEqual(score["missing_truth_rows"], 0)
        self.assertTrue(score["finite"])
        self.assertEqual(score["over_70_mps_count"], 0)

    def test_solution_seal_has_no_coordinate_fields(self):
        route = self.contract.ROUTES[1]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "solution.csv"
            path.write_text(
                "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                f"{route},1000,34.0000000,-118.0000000\n",
                encoding="utf-8",
            )
            metadata = self.contract.seal_solution_metadata(route, path)
        self.assertTrue(metadata["present"])
        self.assertEqual(metadata["rows"], 1)
        self.assertTrue(metadata["sealed_after_native_exit"])
        self.assertFalse(metadata["coordinates_published_in_metadata"])
        self.assertEqual(metadata["truth_reads"], 0)
        self.assertNotIn("latitude", metadata)
        self.assertNotIn("longitude", metadata)

    def test_duplicate_prediction_fails_closed(self):
        p82 = self.contract._import_phase82()
        route = self.contract.ROUTES[0]
        duplicate = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            f"{route},1000,37.0,-122.0\n"
            f"{route},1000,37.0,-122.0\n"
        ).encode()
        with self.assertRaises(Exception):
            p82.P76.P74._parse_submission(duplicate, route)

    def test_baselines_are_same_route_only_and_phase82_macro_is_not_four_route(self):
        freeze = self.contract.verify_freeze()
        self.assertTrue(freeze["comparison_baselines"]["phase82_same_route_candidate"]["official_four_route_macro_direct_comparison"] is False)
        self.assertEqual(freeze["routes"][self.contract.ROUTES[0]]["phase82_same_route_score_m"], 1.1139384500152307)
        self.assertEqual(freeze["routes"][self.contract.ROUTES[1]]["phase100_qr_scalar_clock_score_m"], 3.3204375254720286)

    def test_pre_raw_reports_zero_activity(self):
        pre_raw = self.contract.verify_pre_raw()
        self.assertEqual(pre_raw["raw_reads"], 0)
        self.assertEqual(pre_raw["native_solver_invocations"], 0)
        self.assertEqual(pre_raw["truth_reads"], 0)
        self.assertFalse(pre_raw["raw_content_copied_or_transformed"])


if __name__ == "__main__":
    unittest.main()
