"""Launch-free Phase183 scorer wiring tests; no candidate/truth payload reads."""

from __future__ import annotations

import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase183_phase182_accuracy.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = _load(RUNNER_PATH, "phase183_accuracy_test_runner")


class Phase183AccuracyTests(unittest.TestCase):
    def test_manifest_selects_sealed_phase182_h_only(self) -> None:
        manifest = RUNNER.verify_manifest()
        self.assertEqual(manifest["route_order"], [RUNNER.ROUTE])
        self.assertEqual(manifest["candidate_reference"]["sha256"], RUNNER.H_CANDIDATE_SHA)
        self.assertIn("phase182-h-native-raw-p-no-doppler-imu-main-v1", manifest["candidate_reference"]["path"])
        self.assertNotIn("phase144", manifest["candidate_reference"]["path"])
        self.assertEqual(manifest["truth_reference"]["sha256"], RUNNER.H_TRUTH_SHA)

    def test_metric_contract_is_identical_pinned_phase142_reference(self) -> None:
        manifest = RUNNER.verify_manifest()
        self.assertEqual(manifest["metric_contract"], RUNNER.metric_contract())
        self.assertEqual(manifest["metric_contract"]["earth_radius_m"], 6371008.8)
        self.assertEqual(manifest["metric_contract"]["route_scalar"], "(P50 + P95) / 2 in metres")

    def test_warmup_policy_and_no_double_pixel5_offset(self) -> None:
        manifest = RUNNER.verify_manifest()
        candidate = manifest["candidate_reference"]
        self.assertEqual(candidate["modeled_epochs"], 3140)
        self.assertEqual(candidate["rows"], 3139)
        self.assertTrue(candidate["warmup_epoch_excluded"])
        self.assertEqual(candidate["pixel5_offset_reapplication"], 0)
        self.assertEqual(manifest["truth_reference"]["rows"], 3139)
        self.assertIsNone(manifest["truth_reference"]["expected_missing_truth_key"])

    def test_strict_macro_threshold_and_finite_gate(self) -> None:
        self.assertTrue(RUNNER.strict_macro_gate(0.781999999))
        self.assertFalse(RUNNER.strict_macro_gate(0.782))
        self.assertFalse(RUNNER.strict_macro_gate(float("nan")))
        good = {"prediction_domain_coverage": 1.0, "truth_row_coverage": 1.0, "matched_rows": 3139, "finite": True, "over_70_mps_count": 0, "score_m": 0.1}
        self.assertTrue(all(RUNNER.route_gates(3139, good).values()))
        for bad in ({**good, "prediction_domain_coverage": 0.99}, {**good, "truth_row_coverage": 0.99}, {**good, "finite": False}, {**good, "over_70_mps_count": 1}, {**good, "score_m": float("inf")}):
            self.assertFalse(all(RUNNER.route_gates(3139, bad).values()))
        self.assertFalse(all(RUNNER.route_gates(3138, good).values()))

    def test_duplicate_json_and_payload_materialization_are_fail_closed(self) -> None:
        with tempfile.NamedTemporaryFile("w", suffix=".json", encoding="utf-8") as handle:
            handle.write('{"route": 1, "route": 2}')
            handle.flush()
            with self.assertRaises(RUNNER.Phase183Error):
                RUNNER.read_json(Path(handle.name), "synthetic duplicate")
        with self.assertRaises(RUNNER.Phase183Error):
            RUNNER.static_hash(ROOT / "output/smartphone-r5/phase182-h-native-raw-p-no-doppler-imu-main-v1/mtv-h/opaque_solution_output.csv", "candidate payload")

    def test_exact_reference_parser_join_haversine_and_linear_percentiles(self) -> None:
        _, p82 = RUNNER.metric_reference()
        route = RUNNER.ROUTE
        candidate_payload = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" + "\n".join(
            f"{route},{timestamp},{latitude},0.0" for timestamp, latitude in ((1000, 0.0001), (2000, 0.0002), (3000, 0.0004), (4000, 0.0008))
        ) + "\n").encode()
        truth_payload = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" + "\n".join(
            f"{route},{timestamp},0.0,0.0" for timestamp in (1000, 2000, 3000, 4000)
        ) + "\n").encode()
        ordered, candidate = p82.P76.P74._parse_submission(candidate_payload, route)
        truth = p82.P76._parse_truth_dictreader(truth_payload, route)
        score = p82.P76._score_prediction(candidate, truth, None, route, ordered)
        distances = [
            2.0 * 6371008.8 * math.asin(math.sqrt(math.sin(math.radians(latitude) / 2.0) ** 2))
            for latitude in (0.0001, 0.0002, 0.0004, 0.0008)
        ]
        distances.sort()
        def percentile(q: float) -> float:
            rank = (len(distances) - 1) * q
            lo = int(math.floor(rank))
            hi = int(math.ceil(rank))
            return distances[lo] if lo == hi else distances[lo] + (distances[hi] - distances[lo]) * (rank - lo)
        expected_p50 = percentile(0.50)
        expected_p95 = percentile(0.95)
        self.assertEqual(score["matched_rows"], 4)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertEqual(score["truth_row_coverage"], 1.0)
        self.assertAlmostEqual(score["p50_m"], expected_p50, places=10)
        self.assertAlmostEqual(score["p95_m"], expected_p95, places=10)
        self.assertAlmostEqual(score["score_m"], (expected_p50 + expected_p95) / 2.0, places=10)

    def test_exact_reference_rejects_duplicate_and_extra_keys(self) -> None:
        _, p82 = RUNNER.metric_reference()
        route = RUNNER.ROUTE
        duplicate = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" f"{route},1000,0,0\n{route},1000,0,0\n").encode()
        with self.assertRaises(Exception):
            p82.P76.P74._parse_submission(duplicate, route)
        candidate_payload = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" f"{route},1000,0,0\n{route},2000,0,0\n").encode()
        truth_payload = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" f"{route},1000,0,0\n").encode()
        ordered, candidate = p82.P76.P74._parse_submission(candidate_payload, route)
        truth = p82.P76._parse_truth_dictreader(truth_payload, route)
        with self.assertRaises(Exception):
            p82.P76._score_prediction(candidate, truth, None, route, ordered)


if __name__ == "__main__":
    unittest.main()
