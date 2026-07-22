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

    def test_should_demote_supports_low_satellite_ratio_pair(self) -> None:
        low_satellite_low_ratio = (
            "2300 10.000 1 2 3 0 0 0 4 11 2.0 15.0 100.0 "
            "2 40 20 20 0 0.5 2.0 0.5 2.0 8.0 0.1 0"
        ).split()
        high_ratio = low_satellite_low_ratio.copy()
        high_ratio[11] = "15.1"
        many_satellites = low_satellite_low_ratio.copy()
        many_satellites[9] = "12"
        rule = status_demotion.DemotionRule(
            max_ratio=None,
            min_baseline_m=None,
            max_baseline_m=None,
            max_nis_per_obs=None,
            max_post_rms_m=None,
            min_satellites=8,
            low_satellite_ceiling=11,
            low_satellite_max_ratio=15.0,
        )

        self.assertTrue(status_demotion.should_demote(low_satellite_low_ratio, rule))
        self.assertFalse(status_demotion.should_demote(high_ratio, rule))
        self.assertFalse(status_demotion.should_demote(many_satellites, rule))

    def test_strong_telemetry_exonerates_base_gate(self) -> None:
        parts = (
            "2300 10.000 1 2 3 0 0 0 4 11 2.0 10.0 100.0 "
            "2 40 20 20 0 0.4 2.0 0.2 2.0 8.0 0.1 0"
        ).split()
        rule = status_demotion.DemotionRule(
            max_ratio=None,
            min_baseline_m=None,
            max_baseline_m=None,
            max_nis_per_obs=None,
            max_post_rms_m=None,
            min_satellites=8,
            low_satellite_ceiling=11,
            low_satellite_max_ratio=15.0,
            exonerate_min_satellites=11,
            exonerate_max_prefit_rms_m=0.5,
            exonerate_max_nis_per_obs=0.2,
        )

        self.assertTrue(status_demotion.should_demote(parts, rule))
        self.assertTrue(status_demotion.should_exonerate(parts, rule))

    def test_kinematic_trigger_demotes_bounded_burst(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_kinematic_gate_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            rows = []
            for tow, x in ((10.0, 0.0), (10.2, 0.0), (10.4, 20.0), (10.6, 20.1)):
                rows.append(
                    f"2300 {tow:.3f} {x} 2 3 0 0 0 4 18 2.0 5.0 100.0 "
                    "2 40 20 20 0 0.5 2.0 0.5 2.0 8.0 0.1 0"
                )
            source.write_text("\n".join(rows) + "\n", encoding="ascii")
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                kinematic_max_jump_m=12.0,
                kinematic_min_acceleration_mps2=200.0,
                kinematic_hold_epochs=2,
            )

            summary = status_demotion.apply_file(source, output, rule)

            self.assertEqual(summary.demoted_epochs, 2)
            self.assertEqual(summary.kinematic_demoted_epochs, 2)
            statuses = [
                line.split()[8] for line in output.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(statuses, ["4", "4", "3", "3"])

    def test_kinematic_plateau_and_secondary_trigger_extend_quarantine(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_kinematic_plateau_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            rows = []
            for tow, x in (
                (10.0, 0.0),
                (10.2, 0.0),
                (10.4, 6.0),
                (10.6, 6.01),
                (10.8, 6.02),
                (11.0, 7.0),
            ):
                rows.append(
                    f"2300 {tow:.3f} {x} 2 3 0 0 0 4 13 2.0 8.0 100.0 "
                    "2 40 20 20 12 6.0 2.0 0.5 2.0 8.0 0.4 0"
                )
            source.write_text("\n".join(rows) + "\n", encoding="ascii")
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                kinematic_max_jump_m=12.0,
                kinematic_min_acceleration_mps2=200.0,
                kinematic_hold_epochs=2,
                kinematic_plateau_max_jump_m=0.1,
                kinematic_max_hold_epochs=5,
                kinematic_secondary_min_jump_m=5.0,
                kinematic_secondary_min_acceleration_mps2=100.0,
                kinematic_secondary_min_prefit_rms_m=5.0,
                kinematic_secondary_max_ratio=10.0,
                kinematic_secondary_min_outliers=10,
                kinematic_secondary_max_satellites=13,
            )

            status_demotion.apply_file(source, output, rule)

            statuses = [
                line.split()[8] for line in output.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(statuses, ["4", "4", "3", "3", "3", "4"])

    def test_residual_streak_demotes_only_after_causal_threshold(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_streak_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            rows = []
            for index in range(10):
                prefit = 20.0 if index != 8 else 1.0
                rows.append(
                    f"2300 {10.0 + 0.2 * index:.3f} {index} 2 3 0 0 0 4 15 "
                    f"2.0 10.0 100.0 2 40 20 20 12 {prefit} 2.0 0.5 2.0 8.0 0.4 0"
                )
            source.write_text("\n".join(rows) + "\n", encoding="ascii")
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                residual_streak_min_prefit_rms_m=15.0,
                residual_streak_max_ratio=15.0,
                residual_streak_min_outliers=10,
                residual_streak_epochs=8,
            )

            summary = status_demotion.apply_file(source, output, rule)

            statuses = [
                line.split()[8] for line in output.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(statuses, ["4"] * 7 + ["3", "4", "4"])
            self.assertEqual(summary.residual_streak_demoted_epochs, 1)

    def test_residual_streak_can_buffer_and_demote_confirmed_prefix(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_buffer_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            rows = [
                f"2300 {10.0 + 0.2 * index:.3f} {index} 2 3 0 0 0 4 15 "
                "2.0 10.0 100.0 2 40 20 20 12 20.0 2.0 0.5 2.0 8.0 0.4 0"
                for index in range(9)
            ]
            source.write_text("\n".join(rows) + "\n", encoding="ascii")
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                residual_streak_min_prefit_rms_m=15.0,
                residual_streak_max_ratio=15.0,
                residual_streak_min_outliers=10,
                residual_streak_epochs=8,
                residual_streak_buffer_prefix=True,
            )

            summary = status_demotion.apply_file(source, output, rule)

            statuses = [
                line.split()[8] for line in output.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(statuses, ["3"] * 9)
            self.assertEqual(summary.demoted_epochs, 9)
            self.assertEqual(summary.residual_streak_demoted_epochs, 9)

    def test_residual_spike_requires_high_prefit_and_low_satellite_count(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_spike_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            source.write_text(
                "\n".join(
                    [
                        "2300 10.000 1 2 3 0 0 0 4 14 2.0 20.0 100.0 2 40 20 20 2 40.0 80.0 0.5 2.0 8.0 0.4 0",
                        "2300 10.200 1 2 3 0 0 0 4 15 2.0 20.0 100.0 2 40 20 20 2 50.0 80.0 0.5 2.0 8.0 0.4 0",
                        "2300 10.400 1 2 3 0 0 0 4 14 2.0 20.0 100.0 2 40 20 20 2 39.9 80.0 0.5 2.0 8.0 0.4 0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                residual_spike_min_prefit_rms_m=40.0,
                residual_spike_max_satellites=14,
            )

            summary = status_demotion.apply_file(source, output, rule)

            statuses = [
                line.split()[8] for line in output.read_text(encoding="ascii").splitlines()
            ]
            self.assertEqual(statuses, ["3", "4", "4"])
            self.assertEqual(summary.residual_spike_demoted_epochs, 1)

    def test_residual_summary_counts_rule_overlap_without_double_demotion(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_overlap_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            row = (
                "2300 {tow:.3f} 1 2 3 0 0 0 4 14 2.0 10.0 100.0 "
                "2 40 20 20 12 50.0 80.0 0.5 2.0 8.0 0.4 0"
            )
            source.write_text(
                "\n".join(row.format(tow=10.0 + 0.2 * index) for index in range(8))
                + "\n",
                encoding="ascii",
            )
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                residual_streak_min_prefit_rms_m=40.0,
                residual_streak_max_ratio=15.0,
                residual_streak_min_outliers=12,
                residual_streak_epochs=8,
                residual_streak_buffer_prefix=True,
                residual_spike_min_prefit_rms_m=40.0,
                residual_spike_max_satellites=14,
            )

            summary = status_demotion.apply_file(source, output, rule)

            self.assertEqual(summary.demoted_epochs, 8)
            self.assertEqual(summary.residual_streak_demoted_epochs, 8)
            self.assertEqual(summary.residual_spike_demoted_epochs, 8)
            self.assertEqual(summary.residual_overlap_epochs, 8)

    def test_residual_streak_requires_configured_outlier_fraction(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_fraction_") as temp_dir:
            temp_root = Path(temp_dir)
            source = temp_root / "input.pos"
            output = temp_root / "output.pos"
            row = (
                "2300 {tow:.3f} 1 2 3 0 0 0 4 20 2.0 10.0 100.0 "
                "2 40 20 20 12 50.0 80.0 0.5 2.0 8.0 0.4 0"
            )
            source.write_text(
                "\n".join(row.format(tow=10.0 + 0.2 * index) for index in range(8))
                + "\n",
                encoding="ascii",
            )
            rule = status_demotion.DemotionRule(
                max_ratio=None,
                min_baseline_m=None,
                max_baseline_m=None,
                max_nis_per_obs=None,
                max_post_rms_m=None,
                residual_streak_min_prefit_rms_m=40.0,
                residual_streak_max_ratio=15.0,
                residual_streak_min_outliers=12,
                residual_streak_min_outlier_fraction=0.5,
                residual_streak_epochs=8,
                residual_streak_buffer_prefix=True,
            )

            summary = status_demotion.apply_file(source, output, rule)

            self.assertEqual(summary.demoted_epochs, 0)
            self.assertEqual(summary.residual_streak_demoted_epochs, 0)

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
