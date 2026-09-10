#!/usr/bin/env python3
"""Synthetic contract tests for the Phase189 one-route evaluator."""

from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
RUNNER_PATH = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase189_phase188_h_accuracy.py"
ROUTE = "2021-08-24-20-32-us-ca-mtv-h/pixel5"


def load_runner():
    spec = importlib.util.spec_from_file_location("phase189_accuracy_runner_tests", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError("unable to load Phase189 evaluator")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RUNNER = load_runner()
P76, P74 = RUNNER.load_helpers()


def candidate_payload(rows: list[tuple[int, float, float]], phone: str = ROUTE) -> bytes:
    lines = ["phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees"]
    lines.extend(f"{phone},{timestamp},{latitude},{longitude}" for timestamp, latitude, longitude in rows)
    return ("\n".join(lines) + "\n").encode()


def truth_payload(rows: list[tuple[int, float, float]], extra: bool = False) -> bytes:
    header = "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,Unused" if extra else "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees"
    lines = [header]
    for timestamp, latitude, longitude in rows:
        lines.append(f"{timestamp},{latitude},{longitude},ok" if extra else f"{timestamp},{latitude},{longitude}")
    return ("\n".join(lines) + "\n").encode()


def pin(path: Path, payload: bytes, rows: int) -> dict[str, object]:
    path.write_bytes(payload)
    return {"path": str(path), "sha256": hashlib.sha256(payload).hexdigest(), "bytes": len(payload), "rows": rows}


class Phase189AccuracyContractTest(unittest.TestCase):
    def score(self, candidate: bytes, truth: bytes, candidate_rows: int | None = None, truth_rows: int | None = None):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            candidate_pin = pin(root / "opaque_solution_output.csv", candidate, candidate_rows or len(candidate.splitlines()) - 1)
            truth_pin = pin(root / "ground_truth.csv", truth, truth_rows or len(truth.splitlines()) - 1)
            return RUNNER.score_payloads(
                Path(candidate_pin["path"]), candidate_pin,
                Path(truth_pin["path"]), truth_pin, P76, P74,
                route=ROUTE,
            )

    def test_metric_exact_haversine_linear_percentiles_and_full_domain(self):
        truth = [(10000 * i, 0.0, 0.0) for i in range(1, 5)]
        candidate = [(timestamp, 0.0, 0.001 * i) for i, (timestamp, _, _) in enumerate(truth)]
        result = self.score(candidate_payload(candidate), truth_payload(truth, extra=True))
        unit = RUNNER.EARTH_RADIUS_M * 3.141592653589793 / 180.0
        self.assertAlmostEqual(result["metric"]["p50_m"], unit * 0.0015, places=9)
        self.assertAlmostEqual(result["metric"]["p95_m"], unit * 0.00285, places=9)
        self.assertAlmostEqual(result["metric"]["route_score_m"], unit * 0.002175, places=9)
        self.assertTrue(result["gates"]["candidate_prediction_domain_exact"])
        self.assertTrue(result["gates"]["truth_domain_exact"])
        self.assertTrue(result["gates"]["velocity_gate_over_70_mps_zero"])

    def test_exact_join_rejects_missing_truth_key(self):
        candidate = candidate_payload([(10000, 0.0, 0.0)])
        truth = truth_payload([(10000, 0.0, 0.0), (20000, 0.0, 0.0)])
        with self.assertRaises(RUNNER.Phase189Error):
            self.score(candidate, truth)

    def test_exact_join_rejects_prediction_extra_key(self):
        candidate = candidate_payload([(10000, 0.0, 0.0), (20000, 0.0, 0.0)])
        truth = truth_payload([(10000, 0.0, 0.0)])
        with self.assertRaises(RUNNER.Phase189Error):
            self.score(candidate, truth)

    def test_no_interpolation_or_hold_is_enforced(self):
        candidate = candidate_payload([(10000, 0.0, 0.0), (30000, 0.0, 0.0)])
        truth = truth_payload([(10000, 0.0, 0.0), (20000, 0.0, 0.0), (30000, 0.0, 0.0)])
        with self.assertRaises(RUNNER.Phase189Error):
            self.score(candidate, truth)

    def test_pixel5_offset_is_not_reapplied(self):
        result = self.score(
            candidate_payload([(10000, 37.0, -122.0)]),
            truth_payload([(10000, 37.0, -122.0)]),
        )
        self.assertEqual(result["metric"]["route_score_m"], 0.0)
        self.assertEqual(result["read_accounting"]["pixel5_offset_reapplications"], 0)

    def test_finite_earth_and_duplicate_key_contracts(self):
        with self.assertRaises(Exception):
            P74._parse_submission(
                candidate_payload([(10000, 91.0, 0.0)]), ROUTE
            )
        duplicate = ("phone,UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
                     f"{ROUTE},10000,0,0\n{ROUTE},10000,0,0\n").encode()
        with self.assertRaises(Exception):
            P74._parse_submission(duplicate, ROUTE)
        duplicate_truth_header = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,LatitudeDegrees\n"
            "10000,0,0,0\n"
        ).encode()
        with self.assertRaises(Exception):
            P76._parse_truth_dictreader(duplicate_truth_header, ROUTE)
        nonfinite_truth = (
            "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees\n"
            "10000,nan,0\n"
        ).encode()
        with self.assertRaises(Exception):
            P76._parse_truth_dictreader(nonfinite_truth, ROUTE)

    def test_velocity_gate_is_reported_without_dropping_rows(self):
        candidate = candidate_payload([(1000, 0.0, 0.0), (2000, 0.0, 0.001)])
        truth = truth_payload([(1000, 0.0, 0.0), (2000, 0.0, 0.0)])
        result = self.score(candidate, truth)
        self.assertGreater(result["metric"]["over_70_mps_count"], 0)
        self.assertFalse(result["gates"]["velocity_gate_over_70_mps_zero"])
        self.assertEqual(result["metric"]["matched_rows"], 2)

    def test_candidate_hash_fails_before_truth_read(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            candidate_path = root / "opaque_solution_output.csv"
            candidate_path.write_bytes(candidate_payload([(10000, 0.0, 0.0)]))
            missing_truth = root / "ground_truth.csv"
            bad_pin = {"path": str(candidate_path), "sha256": "0" * 64, "bytes": candidate_path.stat().st_size, "rows": 1}
            truth_pin = {"path": str(missing_truth), "sha256": "1" * 64, "bytes": 1, "rows": 1}
            with self.assertRaisesRegex(RUNNER.Phase189Error, "candidate SHA"):
                RUNNER.score_payloads(candidate_path, bad_pin, missing_truth, truth_pin, P76, P74, route=ROUTE)

    def test_truth_hash_fails_after_candidate_but_before_score(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            candidate_bytes = candidate_payload([(10000, 0.0, 0.0)])
            truth_bytes = truth_payload([(10000, 0.0, 0.0)])
            candidate_path = root / "opaque_solution_output.csv"
            truth_path = root / "ground_truth.csv"
            candidate_path.write_bytes(candidate_bytes)
            truth_path.write_bytes(truth_bytes)
            candidate_pin = {"path": str(candidate_path), "sha256": hashlib.sha256(candidate_bytes).hexdigest(), "bytes": len(candidate_bytes), "rows": 1}
            truth_pin = {"path": str(truth_path), "sha256": "2" * 64, "bytes": len(truth_bytes), "rows": 1}
            with self.assertRaisesRegex(RUNNER.Phase189Error, "truth SHA"):
                RUNNER.score_payloads(candidate_path, candidate_pin, truth_path, truth_pin, P76, P74, route=ROUTE)

    def test_existing_result_rejected_before_payload_reads(self):
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            existing = root / "result.json"
            existing.write_text("sealed\n", encoding="utf-8")
            candidate = root / "opaque_solution_output.csv"
            truth = root / "ground_truth.csv"
            candidate_pin = {"path": str(candidate), "sha256": "0" * 64, "bytes": 1, "rows": 1}
            truth_pin = {"path": str(truth), "sha256": "1" * 64, "bytes": 1, "rows": 1}
            with self.assertRaisesRegex(RUNNER.Phase189Error, "overwrite"):
                RUNNER.score_payloads(candidate, candidate_pin, truth, truth_pin, P76, P74, route=ROUTE, result_path=existing)

    def test_kernel_accepts_another_explicit_route_without_constant_retyping(self):
        route = "synthetic-route/pixel5"
        candidate = candidate_payload([(10000, 35.0, -120.0)], phone=route)
        truth = truth_payload([(10000, 35.0, -120.0)])
        with tempfile.TemporaryDirectory() as raw:
            root = Path(raw)
            candidate_pin = pin(root / "opaque_solution_output.csv", candidate, 1)
            truth_pin = pin(root / "ground_truth.csv", truth, 1)
            result = RUNNER.score_payloads(
                Path(candidate_pin["path"]), candidate_pin,
                Path(truth_pin["path"]), truth_pin, P76, P74,
                route=route,
            )
        self.assertEqual(result["route"], route)
        self.assertEqual(result["metric"]["route_score_m"], 0.0)


if __name__ == "__main__":
    unittest.main()
