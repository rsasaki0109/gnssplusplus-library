from __future__ import annotations

import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase29_train_eval.py"
SPEC = importlib.util.spec_from_file_location("phase29_train_eval", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Phase29TrainEvaluationTests(unittest.TestCase):
    def _truth(self, path: Path, rows: list[tuple[int, float, float, float]], phone: str | None = None) -> None:
        fields = ["UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees", "AltitudeMeters"]
        if phone is not None:
            fields.insert(0, "phone")
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(fields)
            for timestamp, latitude, longitude, altitude in rows:
                values = [timestamp, latitude, longitude, altitude]
                if phone is not None:
                    values.insert(0, phone)
                writer.writerow(values)

    def _prediction(self, path: Path, rows: list[tuple[str, int, float, float]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"])
            writer.writerows(rows)

    def test_current_sealed_routes_and_hashes_verify_without_truth(self) -> None:
        freeze = MODULE._verify_freeze(MODULE.FREEZE)
        self.assertEqual(tuple(freeze["identities"]["development_train"]), MODULE.ROUTES)
        self.assertFalse(freeze["authorization"]["truth_materialized_before_freeze"])

    def test_truth_is_read_once_and_duplicate_keys_fail(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase29-truth-") as temporary:
            path = Path(temporary) / "ground_truth.csv"
            self._truth(path, [(1000, 37.0, -122.0, 10.0), (2000, 37.1, -122.1, 11.0)])
            values, digest, size = MODULE._read_truth_once(path, "route/phone")
            self.assertEqual(len(values), 2)
            self.assertEqual(size, path.stat().st_size)
            self.assertEqual(len(digest), 64)
            path.write_text(
                "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n"
                "1000,37,-122,10\n1000,37,-122,10\n",
                encoding="utf-8",
            )
            with self.assertRaises(MODULE.Phase29Error):
                MODULE._read_truth_once(path, "route/phone")

    def test_common_key_set_and_unavailable_vertical_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase29-score-") as temporary:
            root = Path(temporary)
            truth_path = root / "truth.csv"
            self._truth(truth_path, [(1000, 37.0, -122.0, 10.0), (2000, 37.0, -122.0, 10.0)])
            truth, _digest, _size = MODULE._read_truth_once(truth_path, "route/phone")
            control_path = root / "control.csv"
            candidate_path = root / "candidate.csv"
            self._prediction(control_path, [("route/phone", 1000, 37.0, -122.0), ("route/phone", 2000, 37.0, -122.0)])
            self._prediction(candidate_path, [("route/phone", 1000, 37.0, -122.0), ("route/phone", 2000, 37.0, -122.0)])
            control = MODULE._lane_metrics(MODULE._prediction_rows(control_path, "route/phone"), truth, set(truth))
            candidate = MODULE._lane_metrics(MODULE._prediction_rows(candidate_path, "route/phone"), truth, set(truth))
            self.assertEqual(control["shared_scored_rows"], 2)
            self.assertIsNone(candidate["vertical_p95_abs_m"])
            gate = MODULE._compare(candidate, control)
            self.assertFalse(gate["passed"])
            self.assertIn("vertical_metric_unavailable", gate["failures"])

    def test_duplicate_prediction_and_phone_mismatch_fail_closed(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase29-prediction-") as temporary:
            root = Path(temporary)
            duplicate = root / "duplicate.csv"
            self._prediction(duplicate, [("route/phone", 1000, 37.0, -122.0), ("route/phone", 1000, 37.0, -122.0)])
            with self.assertRaises(MODULE.Phase29Error):
                MODULE._prediction_rows(duplicate, "route/phone")
            mismatch = root / "mismatch.csv"
            self._prediction(mismatch, [("other/phone", 1000, 37.0, -122.0)])
            with self.assertRaises(MODULE.Phase29Error):
                MODULE._prediction_rows(mismatch, "route/phone")

    def test_continuity_reports_physical_bound_crossing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase29-continuity-") as temporary:
            path = Path(temporary) / "candidate.csv"
            self._prediction(path, [("route/phone", 1000, 37.0, -122.0), ("route/phone", 2000, 37.01, -122.0)])
            continuity = MODULE._transition_speed(MODULE._prediction_rows(path, "route/phone"))
            self.assertEqual(continuity["transition_count"], 1)
            self.assertGreater(continuity["over_70_mps_count"], 0)


if __name__ == "__main__":
    unittest.main()
