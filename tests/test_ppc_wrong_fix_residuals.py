#!/usr/bin/env python3
"""Tests for PPC residual wrong-FIX analysis."""

from __future__ import annotations

import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import analyze_ppc_wrong_fix_residuals as wrong_fix  # noqa: E402
import build_ppc_wrong_fix_event_ledger as event_ledger  # noqa: E402


class PPCWrongFixResidualsTest(unittest.TestCase):
    def test_event_ledger_splits_clusters_and_classifies_catastrophic_basin(self) -> None:
        def labeled(tow_s: float, error_m: float) -> event_ledger.LabeledEpoch:
            return event_ledger.LabeledEpoch(
                wrong_fix.SolutionEpoch(
                    week=2300,
                    tow_s=tow_s,
                    ecef=(0.0, 0.0, 0.0),
                    status=4,
                    nsat=18,
                    ratio=3.0,
                    baseline_m=9000.0,
                    outliers=40,
                    prefit_rms_m=12.0,
                    prefit_max_m=50.0,
                    post_rms_m=0.8,
                    post_max_m=2.5,
                    nis_per_obs=2.0,
                ),
                error_m,
            )

        events = event_ledger.split_events(
            [labeled(10.0, 80.0), labeled(10.2, 81.0), labeled(11.0, 2.0)]
        )
        payload = event_ledger.build_event_payload(
            "nagoya_run3", 1, events[0], [row.solution for row in events[0]], {}
        )

        self.assertEqual(len(events), 2)
        self.assertEqual(payload["severity"]["above_10m"], 2)
        self.assertIn("catastrophic_gt50m", payload["fingerprints"])
        self.assertIn("high_prefit_basin", payload["fingerprints"])
        self.assertIn("outlier_suppression_storm", payload["fingerprints"])
        self.assertIn("low_ar_margin", payload["fingerprints"])
    def test_integrity_rule_requires_low_satellite_and_low_ratio(self) -> None:
        epoch = wrong_fix.SolutionEpoch(
            week=2300,
            tow_s=10.0,
            ecef=(0.0, 0.0, 0.0),
            status=4,
            nsat=11,
            ratio=15.0,
            baseline_m=None,
            outliers=None,
            prefit_rms_m=None,
            prefit_max_m=None,
            post_rms_m=None,
            post_max_m=None,
            nis_per_obs=None,
        )

        self.assertTrue(wrong_fix.integrity_rule_demotes(epoch, 8, 11, 15.0))
        self.assertFalse(
            wrong_fix.integrity_rule_demotes(
                wrong_fix.SolutionEpoch(**{**epoch.__dict__, "ratio": 15.1}), 8, 11, 15.0
            )
        )
        self.assertFalse(
            wrong_fix.integrity_rule_demotes(
                wrong_fix.SolutionEpoch(**{**epoch.__dict__, "nsat": 12}), 8, 11, 15.0
            )
        )

    def test_analyze_profile_counts_source_and_gate_separation(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_wrong_fix_") as temp_dir:
            root = Path(temp_dir)
            dataset_root = root / "PPC-Dataset"
            run_dir = dataset_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            run_dir.joinpath("reference.csv").write_text(
                "\n".join(
                    [
                        "GPS Week,GPS TOW (s),ECEF X (m),ECEF Y (m),ECEF Z (m)",
                        "2300,10.000,1000.0,2000.0,3000.0",
                        "2300,10.200,1001.0,2001.0,3001.0",
                        "2300,10.400,1002.0,2002.0,3002.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            solution_dir = root / "profile"
            solution_dir.mkdir()
            solution_dir.joinpath("tokyo_run1.pos").write_text(
                "\n".join(
                    [
                        "% synthetic",
                        "2300 10.000 1000.000 2000.000 3000.000 0 0 0 4 18 2.0 10.0 100.0 2 40 20 20 0 0.5 2.0 0.5 2.0 40.0 1.0 0",
                        "2300 10.200 1011.000 2001.000 3001.000 0 0 0 4 14 2.0 4.0 8000.0 2 40 20 20 1 6.0 12.0 5.0 18.0 1200.0 30.0 0",
                        "2300 10.400 1002.200 2002.000 3002.000 0 0 0 3 12 2.0 0.0 100.0 2 40 20 20 0 0.5 2.0 0.5 2.0 10.0 0.25 0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            solution_dir.joinpath("tokyo_run1_segments.csv").write_text(
                "\n".join(
                    [
                        "reference_index,start_tow_s,end_tow_s,selected_candidate,rule_matched,score_delta_distance_m,segment_distance_m,score_transition,status_transition,baseline_status_name,selected_status_name,selected_solution_tow_s",
                        "1,10.0,10.2,baseline,False,0.0,0.0,,,,,10.2",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            payload = wrong_fix.analyze_profile(
                wrong_fix.ProfileSpec("profile", solution_dir, solution_dir),
                dataset_root,
                0.50,
            )

            self.assertEqual(payload["fixed"], 2)
            self.assertEqual(payload["wrong_fix"], 1)
            self.assertEqual(payload["wrong_error_severity"]["above_5m"], 1)
            self.assertEqual(payload["selected_candidate_counts"], {"baseline": 1})
            self.assertEqual(payload["rule_matched_counts"], {"false": 1})
            gates = {row["gate"]: row for row in payload["gate_simulation"]}
            self.assertEqual(gates["post_rms > 4 m"]["wrong_caught"], 1)
            self.assertEqual(gates["post_rms > 4 m"]["good_harmed"], 0)
            self.assertEqual(gates["ratio < 6"]["wrong_caught"], 1)
            self.assertEqual(gates["ratio < 6"]["good_harmed"], 0)

    def test_render_markdown_includes_profile_source_summary(self) -> None:
        markdown = wrong_fix.render_markdown(
            {
                "wrong_fix_threshold_m": 0.5,
                "profiles": [
                    {
                        "name": "profile",
                        "fixed": 2,
                        "wrong_fix": 1,
                        "wrong_fix_rate_pct": 50.0,
                        "selected_candidate_counts": {"baseline": 1},
                        "rule_matched_counts": {"false": 1},
                        "runs": [
                            {
                                "key": "tokyo_run1",
                                "available": True,
                                "fixed": 2,
                                "wrong_fix": 1,
                                "wrong_fix_rate_pct": 50.0,
                                "wrong_error_p95_m": 10.0,
                                "wrong_error_severity": {
                                    "above_1m": 1,
                                    "above_2m": 1,
                                    "above_5m": 1,
                                    "above_10m": 0,
                                },
                                "longest_wrong_spans": [
                                    {
                                        "start_tow_s": 10.2,
                                        "end_tow_s": 10.2,
                                        "duration_s": 0.2,
                                        "epochs": 1,
                                    }
                                ],
                            }
                        ],
                        "discriminators": [
                            {
                                "metric": "post_rms_m",
                                "good_fix": {"p50": 0.5, "p90": 0.5, "p95": 0.5},
                                "wrong_fix": {"p50": 5.0, "p90": 5.0, "p95": 5.0},
                            }
                        ],
                        "gate_simulation": [
                            {
                                "gate": "post_rms > 4 m",
                                "wrong_caught": 1,
                                "wrong_caught_pct": 100.0,
                                "good_harmed": 0,
                                "good_harmed_pct": 0.0,
                            }
                        ],
                    }
                ],
            }
        )

        self.assertIn("baseline=1", markdown)
        self.assertIn("post_rms > 4 m", markdown)
        self.assertIn("| 1 | 0 | 10.000 m |", markdown)


if __name__ == "__main__":
    unittest.main()
