"""Launch-free tests for the Phase100 QR accuracy contract.

These tests use only synthetic CSV bytes and committed metadata.  They do not
open raw inputs, official truth, native output, MAT/base data, or Kaggle.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase100_phase99_qr_accuracy.py"
WRAPPER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase100_phase99_qr_accuracy_execute.py"


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase100QrAccuracyContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_module(CONTRACT_PATH, "phase100_qr_accuracy_contract_test")

    def test_freeze_and_pre_raw_accounting_are_closed(self):
        freeze = self.contract.verify_freeze()
        pre_raw = self.contract.verify_pre_raw()
        self.assertEqual(freeze["phase"], 100)
        self.assertEqual(freeze["candidate"]["routes"], list(self.contract.ROUTES))
        self.assertEqual(pre_raw["raw_reads"], 0)
        self.assertEqual(pre_raw["native_solver_invocations"], 0)
        self.assertEqual(pre_raw["truth_reads"], 0)
        self.assertFalse(pre_raw["raw_content_copied_or_transformed"])

    def test_manifest_has_exact_raw_only_qr_commands(self):
        manifest = self.contract.verify_manifest()
        self.assertEqual(
            [item["dataset_id"] for item in manifest["routes"]],
            list(self.contract.ROUTES),
        )
        self.assertEqual(manifest["matrix"]["native_invocations_planned"], 2)
        self.assertEqual(manifest["matrix"]["truth_reads_in_native"], 0)
        for record in manifest["routes"]:
            command = record["command"]
            self.assertEqual(command.count(self.contract.SELECTOR), 1)
            self.assertEqual(command.count(self.contract.HANDOFF_SELECTOR), 1)
            self.assertNotIn("--native-source-clock-c0d-phase94-stage-diagnostics", command)
            self.assertNotIn("--native-source-clock-c0d-phase96-main-diagnostics", command)
            self.assertNotIn("--native-source-clock-c0d-phase98-solver-rank-diagnostic", command)
            self.assertEqual(command[command.index("--out") + 1].endswith("/solution.csv"), True)
            self.assertNotIn("truth", " ".join(command).lower())

    def test_synthetic_metric_matches_phase82_parser_and_score(self):
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
        self.assertGreater(score["score_m"], 0.0)

    def test_solution_seal_contains_no_coordinate_rows(self):
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

    def test_duplicate_prediction_is_rejected_before_truth(self):
        p82 = self.contract._import_phase82()
        route = self.contract.ROUTES[0]
        duplicate = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            f"{route},1000,37.0,-122.0\n"
            f"{route},1000,37.0,-122.0\n"
        ).encode()
        with self.assertRaises(Exception):
            p82.P76.P74._parse_submission(duplicate, route)


if __name__ == "__main__":
    unittest.main()
