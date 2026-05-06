#!/usr/bin/env python3
"""Tests for PPC status demotion helper."""

from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
