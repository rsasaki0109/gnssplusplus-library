"""Launch-free Phase186 evaluator contract and metric tests.

These tests use only in-memory synthetic CSV bytes and sealed metadata.  They
do not open the Phase185 candidate or official truth payloads, launch native
code, or calculate an accuracy score from repository payloads.
"""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase186_phase185_h_accuracy.py"


def load_runner():
    spec = importlib.util.spec_from_file_location("phase186_h_accuracy_runner", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"unable to import {RUNNER_PATH}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class Phase186HAccuracyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.runner = load_runner()
        cls.helpers = cls.runner.load_helpers()

    def test_manifest_pins_phase185_h_and_development_truth_without_payload_reads(self) -> None:
        pre = self.runner.verify_pre_truth()
        self.assertEqual(pre["candidate_payload_reads"], 0)
        self.assertEqual(pre["truth_payload_reads"], 0)
        self.assertEqual(pre["accuracy_calculations"], 0)
        frozen = self.runner.verify_manifest()
        metadata = frozen["metadata"]
        self.assertIn("phase185-h-native-source-tdcp-huber-v1", metadata["candidate"]["path"])
        self.assertEqual(metadata["candidate"]["rows"], 3139)
        self.assertEqual(metadata["candidate"]["modeled_epochs"], 3140)
        self.assertEqual(metadata["truth"]["rows"], 3139)
        self.assertIn("not heldout", metadata["truth"]["role"])
        self.assertFalse(metadata["candidate"]["pixel5_offset_reapplication"])

    def test_known_four_meridian_distances_use_exact_metric_and_linear_quantiles(self) -> None:
        route = self.runner.ROUTE
        latitudes = (0.001, 0.002, 0.004, 0.008)
        rows = "\n".join(f"{route},{t},0.0,0.0" for t in (1000, 2000, 3000, 4000))
        candidate_payload = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" + rows + "\n").encode()
        truth_payload = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            + "\n".join(f"{route},{t},{lat},0.0" for t, lat in zip((1000, 2000, 3000, 4000), latitudes))
            + "\n"
        ).encode()
        ordered, candidate = self.helpers.P74._parse_submission(candidate_payload, route)
        truth = self.helpers._parse_truth_dictreader(truth_payload, route)
        score = self.helpers._score_prediction(candidate, truth, None, route, ordered)
        distances = [self.runner.EARTH_RADIUS_M * math.radians(value) for value in latitudes]
        distances.sort()

        def percentile(q: float) -> float:
            rank = (len(distances) - 1) * q
            lower, upper = math.floor(rank), math.ceil(rank)
            if lower == upper:
                return distances[lower]
            return distances[lower] + (rank - lower) * (distances[upper] - distances[lower])

        p50, p95 = percentile(0.50), percentile(0.95)
        self.assertEqual(score["matched_rows"], 4)
        self.assertEqual(score["prediction_domain_coverage"], 1.0)
        self.assertEqual(score["truth_row_coverage"], 1.0)
        self.assertAlmostEqual(score["p50_m"], p50, places=10)
        self.assertAlmostEqual(score["p95_m"], p95, places=10)
        self.assertAlmostEqual(score["score_m"], (p50 + p95) / 2.0, places=10)

    def test_candidate_duplicate_header_and_key_fail_closed(self) -> None:
        route = self.runner.ROUTE
        duplicate_header = ("phone,UnixTimeMillis,UnixTimeMillis,LongitudeDegrees\n" f"{route},1000,1000,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers.P74._parse_submission(duplicate_header, route)
        duplicate_key = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n" f"{route},1000,0,0\n{route},1000,0,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers.P74._parse_submission(duplicate_key, route)

    def test_missing_and_extra_prediction_keys_fail_closed(self) -> None:
        route = self.runner.ROUTE
        candidate_payload = (
            "phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            f"{route},1000,0,0\n{route},2000,0,0\n"
        ).encode()
        truth_payload = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,quality\n"
            "1000,0,0,ok\n3000,0,0,ok\n"
        ).encode()
        ordered, candidate = self.helpers.P74._parse_submission(candidate_payload, route)
        truth = self.helpers._parse_truth_dictreader(truth_payload, route)
        with self.assertRaises(Exception):
            self.helpers._score_prediction(candidate, truth, None, route, ordered)

    def test_truth_duplicate_header_key_nan_and_range_fail_closed(self) -> None:
        route = self.runner.ROUTE
        duplicate_header = ("UnixTimeMillis,LatitudeDegrees,LatitudeDegrees,LongitudeDegrees\n1000,0,0,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers._parse_truth_dictreader(duplicate_header, route)
        duplicate_key = ("UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n1000,0,0\n1000,0,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers._parse_truth_dictreader(duplicate_key, route)
        nonfinite = ("UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n1000,nan,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers._parse_truth_dictreader(nonfinite, route)
        out_of_range = ("UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n1000,91,0\n").encode()
        with self.assertRaises(Exception):
            self.helpers._parse_truth_dictreader(out_of_range, route)

    def test_hash_mismatch_and_nonempty_output_fail_before_truth(self) -> None:
        with self.assertRaises(self.runner.Phase186Error):
            self.runner.verify_payload_seal(b"abc", {"bytes": 3, "sha256": "0" * 64}, "synthetic candidate")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "result.json"
            output.write_text("existing", encoding="utf-8")
            with self.assertRaises(self.runner.Phase186Error):
                self.runner.ensure_fresh_output(output)

    def test_evaluate_candidate_hash_failure_does_not_read_truth(self) -> None:
        frozen = {
            "metadata": {
                "candidate": {"path": "output/synthetic/opaque_solution_output.csv", "sha256": "a" * 64, "bytes": 3, "rows": 1},
                "truth": {"path": "output/synthetic/truth/ground_truth.csv", "sha256": "b" * 64, "bytes": 3, "rows": 1, "role": "development/train; not heldout or leaderboard proof"},
            }
        }
        with tempfile.TemporaryDirectory() as directory:
            authorization = Path(directory) / "authorization.json"
            authorization.write_text(
                '{"schema_version":"smartphone-r5-phase186-phase185-h-accuracy-authorization.v1",'
                '"manifest_schema":"smartphone-r5-phase186-phase185-h-accuracy-manifest.v1",'
                '"allow_truth_read":true,"manifest_sha256":"%s",'
                '"candidate_sha256":"%s","truth_sha256":"%s"}'
                % (self.runner.static_hash(self.runner.MANIFEST, "manifest"), "a" * 64, "b" * 64),
                encoding="utf-8",
            )
            with mock.patch.object(self.runner, "verify_manifest", return_value=frozen), \
                 mock.patch.object(self.runner, "read_candidate_once", side_effect=self.runner.Phase186Error("candidate SHA-256/byte seal mismatch")), \
                 mock.patch.object(self.runner, "read_truth_once") as read_truth:
                with self.assertRaises(self.runner.Phase186Error):
                    self.runner.evaluate(authorization, Path(directory) / "result.json")
                read_truth.assert_not_called()

    def test_alignment_domain_velocity_and_strict_gate_are_all_required(self) -> None:
        good = {
            "prediction_domain_coverage": 1.0,
            "truth_row_coverage": 1.0,
            "matched_rows": self.runner.TRUTH_ROWS,
            "finite": True,
            "over_70_mps_count": 0,
            "score_m": 0.781999,
        }
        self.assertTrue(all(self.runner.route_gates(self.runner.CANDIDATE_ROWS, good).values()))
        self.assertTrue(self.runner.strict_route_gate(0.781999))
        self.assertFalse(self.runner.strict_route_gate(0.782))
        for bad in (
            {**good, "prediction_domain_coverage": 0.99},
            {**good, "truth_row_coverage": 0.99},
            {**good, "finite": False},
            {**good, "over_70_mps_count": 1},
            {**good, "score_m": float("inf")},
        ):
            self.assertFalse(all(self.runner.route_gates(self.runner.CANDIDATE_ROWS, bad).values()))
        self.assertFalse(all(self.runner.route_gates(self.runner.CANDIDATE_ROWS - 1, good).values()))

    def test_evaluator_is_single_route_and_does_not_reapply_pixel5_offset(self) -> None:
        source = RUNNER_PATH.read_text(encoding="utf-8")
        self.assertNotIn("import subprocess", source)
        self.assertNotIn("Popen", source)
        manifest = self.runner.verify_manifest()["manifest"]
        self.assertEqual(manifest["route_order"], [self.runner.ROUTE])
        self.assertFalse(manifest["metric_contract"]["pixel5_offset_reapplication"])
        self.assertEqual(manifest["comparison_baseline"]["phase183_h_route_score_m"], 1.2874762859947766)
        self.assertIn("not heldout", manifest["comparison_baseline"]["role"])


if __name__ == "__main__":
    unittest.main()
