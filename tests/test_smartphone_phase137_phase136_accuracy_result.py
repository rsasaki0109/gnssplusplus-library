"""Launch-free checks for the sealed Phase137 truth-only result.

Only the tracked aggregate result is read.  Candidate/truth rows and all raw
or native payloads remain outside this test.
"""

from __future__ import annotations

import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
RESULT = ROOT / "docs/use_cases/records/smartphone_r5_phase137_phase136_accuracy_result_v1.json"


class Phase137AccuracyResultTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.result = json.loads(RESULT.read_text(encoding="utf-8"))

    def test_no_go_identity_and_strict_gate(self) -> None:
        self.assertEqual(self.result["schema_version"], "smartphone-r5-phase137-phase136-accuracy-result.v1")
        self.assertEqual(self.result["phase"], 137)
        self.assertEqual(self.result["status"], "no-go-phase137-truth-only-accuracy")
        self.assertEqual(self.result["candidate"], "phase137-phase136-official-affine-opaque-truth-only-accuracy-v1")
        self.assertEqual(self.result["strict_0_782_gate"]["comparator"], "candidate_macro_score_m < 0.782")
        self.assertFalse(self.result["strict_0_782_gate"]["passed"])
        self.assertEqual(self.result["aggregate"]["candidate_macro_score_m"], 266.46573024454483)
        self.assertEqual(self.result["aggregate"]["phase112_macro_m"], 0.8318381724)
        self.assertEqual(self.result["aggregate"]["phase118_champion_macro_m"], 0.8141981503)

    def test_route_scores_and_non_accuracy_gates_are_sealed(self) -> None:
        routes = self.result["routes"]
        expected = {
            "2021-03-16-18-59-us-ca-mtv-a/pixel5": (2158, 335.10051390489855),
            "2022-04-01-18-22-us-ca-lax-t/pixel5": (1465, 197.83094658419105),
        }
        for route, (rows, score) in expected.items():
            with self.subTest(route=route):
                item = routes[route]
                self.assertEqual(item["candidate"]["prediction_rows"], rows)
                self.assertEqual(item["candidate"]["score_m"], score)
                self.assertTrue(item["candidate"]["finite"])
                self.assertEqual(item["candidate"]["prediction_domain_coverage"], 1.0)
                self.assertEqual(item["candidate"]["over_70_mps_count"], 0)
                self.assertFalse(item["gates"]["passed"])
                self.assertIn("candidate_no_route_regression_vs_phase112", item["gates"]["failures"])
                self.assertTrue(item["candidate_solution"]["coordinate_rows_omitted"])
                self.assertTrue(item["truth"]["coordinate_rows_omitted"])
        self.assertEqual(self.result["failed_gates"], [
            "candidate_each_route_score_at_most_3m",
            "candidate_no_route_regression_vs_phase112",
            "candidate_macro_score_strict_less_than_0_782m",
        ])

    def test_exact_read_accounting_and_no_publication(self) -> None:
        accounting = self.result["read_accounting"]
        self.assertEqual(accounting["candidate_solution_reads_for_hash_and_parse"], 2)
        self.assertEqual(accounting["candidate_coordinate_interpretations"], 2)
        self.assertEqual(accounting["candidate_paths_materialized_after_authorization"], 2)
        self.assertEqual(accounting["truth_paths_materialized_after_authorization"], 2)
        self.assertEqual(accounting["truth_reads"], 2)
        self.assertEqual(accounting["truth_reads_per_route"], 1)
        self.assertEqual(accounting["accuracy_calculations"], 2)
        for key in (
            "native_solver_invocations", "raw_gnss_imu_navigation_reads",
            "raw_base_rinex_reads", "reruns", "fallbacks",
            "mat_precomputed_phone_coordinate_pdc_reads", "kaggle_or_token_access",
        ):
            self.assertEqual(accounting[key], 0, key)
        self.assertFalse(self.result["solution_output_published"])
        self.assertFalse(self.result["release_or_submission_authorized"])
        self.assertFalse(self.result["forbidden_lanes"]["solution_rows_in_result"])

    def test_only_opaque_candidate_metadata_is_present(self) -> None:
        for item in self.result["routes"].values():
            self.assertEqual(set(item["candidate_solution"]), {"bytes", "sha256", "rows", "read_count", "coordinate_rows_omitted"})
            self.assertEqual(set(item["truth"]), {"bytes", "sha256", "rows", "read_count", "coordinate_rows_omitted"})
            self.assertEqual(item["candidate_solution"]["read_count"], 1)
            self.assertEqual(item["truth"]["read_count"], 1)
            self.assertEqual(len(item["candidate_solution"]["sha256"]), 64)
            self.assertEqual(len(item["truth"]["sha256"]), 64)


if __name__ == "__main__":
    unittest.main()
