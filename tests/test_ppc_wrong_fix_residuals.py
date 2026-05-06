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


class PPCWrongFixResidualsTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
