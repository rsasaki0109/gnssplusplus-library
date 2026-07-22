#!/usr/bin/env python3
"""Tests for PPC status demotion helper."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from pathlib import Path


ROOT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT_DIR / "scripts"))

import apply_ppc_status_demotion as status_demotion  # noqa: E402


class PPCStatusDemotionTest(unittest.TestCase):
    def test_apply_file_demotes_only_matching_fixed_epochs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_status_demotion_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            source.write_text(
                "\n".join(
                    [
                        "% header",
                        "2300 10.000 1 2 3 0 0 0 4 18 2.0 5.0 8100.0 2 40 20 20 0 0.5 2.0 0.6 2.0 80.0 9.0 0",
                        "2300 10.200 1 2 3 0 0 0 4 18 2.0 7.0 8100.0 2 40 20 20 0 0.5 2.0 0.6 2.0 80.0 9.0 0",
                        "2300 10.400 1 2 3 0 0 0 3 18 2.0 5.0 8100.0 2 40 20 20 0 0.5 2.0 0.6 2.0 80.0 9.0 0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rule = status_demotion.DemotionRule(
                max_ratio=6.0,
                min_baseline_m=8000.0,
                max_baseline_m=8500.0,
                max_nis_per_obs=8.0,
                max_post_rms_m=None,
            )

            summary = status_demotion.apply_file(source, output, rule)

            self.assertEqual(summary.fixed_epochs, 2)
            self.assertEqual(summary.demoted_epochs, 1)
            rows = [
                line.split()
                for line in output.read_text(encoding="ascii").splitlines()
                if line and not line.startswith("%")
            ]
            self.assertEqual(rows[0][8], "3")
            self.assertEqual(rows[1][8], "4")
            self.assertEqual(rows[2][8], "3")

    def test_should_demote_supports_post_rms_rule(self) -> None:
        parts = (
            "2300 10.000 1 2 3 0 0 0 4 18 2.0 20.0 100.0 "
            "2 40 20 20 0 0.5 2.0 4.5 2.0 80.0 1.0 0"
        ).split()
        rule = status_demotion.DemotionRule(
            max_ratio=None,
            min_baseline_m=None,
            max_baseline_m=None,
            max_nis_per_obs=None,
            max_post_rms_m=4.0,
        )

        self.assertTrue(status_demotion.should_demote(parts, rule))

    def test_should_demote_supports_minimum_satellites(self) -> None:
        low_satellites = (
            "2300 10.000 1 2 3 0 0 0 4 8 2.0 20.0 100.0 "
            "2 40 20 20 0 0.5 2.0 0.5 2.0 8.0 0.1 0"
        ).split()
        enough_satellites = low_satellites.copy()
        enough_satellites[9] = "9"
        rule = status_demotion.DemotionRule(
            max_ratio=None,
            min_baseline_m=None,
            max_baseline_m=None,
            max_nis_per_obs=None,
            max_post_rms_m=None,
            min_satellites=9,
        )

        self.assertTrue(status_demotion.should_demote(low_satellites, rule))
        self.assertFalse(status_demotion.should_demote(enough_satellites, rule))

    def test_apply_metrics_resolves_selected_paths(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_status_metrics_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "selected.pos"
            source.write_text(
                "2300 10.000 1 2 3 0 0 0 4 8 2.0 5.0 100.0 "
                "2 40 20 20 0 0.5 2.0 0.5 2.0 8.0 0.1 0\n",
                encoding="ascii",
            )
            metrics = temp_root / "metrics.json"
            metrics.write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "key": "tokyo_run1",
                                "libgnss": {"pos": str(source)},
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            output_dir = temp_root / "output"
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                min_satellites=9,
            )

            summaries = status_demotion.apply_metrics(metrics, output_dir, rule)

            self.assertEqual(len(summaries), 1)
            self.assertEqual(summaries[0].demoted_epochs, 1)
            self.assertEqual(
                (output_dir / "tokyo_run1.pos").read_text(encoding="ascii").split()[8],
                "3",
            )


if __name__ == "__main__":
    unittest.main()
