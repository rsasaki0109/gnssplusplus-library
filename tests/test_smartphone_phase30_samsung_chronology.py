from __future__ import annotations

import csv
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "apps/commands/benchmarks/gnss_smartphone_phase30_samsung_chronology.py"
SPEC = importlib.util.spec_from_file_location("phase30_samsung_chronology", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class Phase30SamsungChronologyTests(unittest.TestCase):
    def _prediction(self, path: Path, rows: list[tuple[int, float, float]]) -> None:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(["phone", "UnixTimeMillis", "LatitudeDegrees", "LongitudeDegrees"])
            for timestamp, latitude, longitude in rows:
                writer.writerow([MODULE.ROUTE, timestamp, latitude, longitude])

    def _raw(self, path: Path, rows: list[list[str]]) -> None:
        fields = list(MODULE.RAW_COLUMNS) + ["RawPseudorangeMeters", "SvElevationDegrees"]
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(fields)
            writer.writerows(rows)

    def test_raw_structural_reader_ignores_enriched_columns_and_tracks_gap(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase30-raw-") as temporary:
            path = Path(temporary) / "device_gnss.csv"
            def row(timestamp: int, time_nanos: int, hcdc: int) -> list[str]:
                values = ["Raw", str(timestamp), str(time_nanos), str(hcdc), "16431", "1", "GPS_L1_CA", "1.0", "25", "35.0"]
                return values + ["999999999.0", "1.0"]
            self._raw(path, [row(1000, 1000000000, 7), row(1000, 1000000000, 7), row(2001, 2001000000, 7), row(3001, 3001000000, 8)])
            epochs, report = MODULE._read_raw_epochs(path)
            self.assertEqual(len(epochs), 3)
            self.assertEqual(report["hcdc_transition_count"], 1)
            self.assertEqual(report["raw_gap_count_over_1000ms"], 1)
            self.assertEqual(report["forbidden_derived_fields_used"], [])
            self.assertIn("RawPseudorangeMeters", report["derived_columns_present_but_ignored"])

    def test_two_lane_linearity_reconciles_sealed_interpolation_count(self) -> None:
        timestamps = [1000, 2000, 3000]
        control = {timestamp: (37.0, -122.0) for timestamp in timestamps}
        candidate = dict(control)
        control_summary = {"raw_utc_key_contract": {"exact_solution_epochs": 2, "interpolated_epochs": 1}}
        candidate_summary = {"raw_utc_key_contract": {"exact_solution_epochs": 2, "interpolated_epochs": 1}}
        rows, report = MODULE._source_lane_inference(control, candidate, control_summary, candidate_summary)
        self.assertEqual(report["inferred_interpolated_epochs"], 1)
        self.assertTrue(report["inference_count_matches_candidate_summary"])
        self.assertEqual(rows[1]["source_lane"], "inferred-interpolated")
        self.assertEqual(rows[0]["source_lane"], "exact-by-sealed-alignment-count")

    def test_truth_reader_uses_one_payload_and_preserves_hash(self) -> None:
        with tempfile.TemporaryDirectory(prefix="phase30-truth-") as temporary:
            path = Path(temporary) / "ground_truth.csv"
            path.write_text(
                "UnixTimeMillis,LatitudeDegrees,LongitudeDegrees,AltitudeMeters\n"
                "1000,37,-122,10\n2000,37.0001,-122.0001,11\n",
                encoding="utf-8",
            )
            truth, digest, size = MODULE._read_truth_once(path, MODULE.ROUTE)
            self.assertEqual(len(truth), 2)
            self.assertEqual(size, path.stat().st_size)
            self.assertEqual(len(digest), 64)

    def test_tail_run_reports_structural_context(self) -> None:
        rows = []
        for index, timestamp in enumerate((1000, 2000, 3000)):
            rows.append(
                {
                    "index": index,
                    "timestamp": timestamp,
                    "source_lane": "exact-by-sealed-alignment-count",
                    "raw": {"raw_clock_segment": 0, "hcdc": 7, "raw_gap_ms": None},
                    "control": {"horizontal_error_m": 600.0 if index == 1 else 1.0, "transition_speed_mps": 10.0},
                    "candidate": {"horizontal_error_m": 650.0 if index == 1 else 1.0, "transition_speed_mps": 11.0},
                }
            )
        runs = MODULE._contiguous_runs(rows, 500.0, "horizontal_error_m")
        self.assertEqual(len(runs), 1)
        self.assertEqual(runs[0]["epoch_count"], 1)
        self.assertEqual(runs[0]["raw_clock_segments"], [0])


if __name__ == "__main__":
    unittest.main()
