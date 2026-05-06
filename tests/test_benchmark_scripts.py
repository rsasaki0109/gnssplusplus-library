#!/usr/bin/env python3
"""Regression tests for benchmark and reporting scripts."""

from __future__ import annotations

import argparse
import csv
from datetime import timedelta
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[1]
APPS_DIR = ROOT_DIR / "apps"
SCRIPTS_DIR = ROOT_DIR / "scripts"
CI_SCRIPTS_DIR = SCRIPTS_DIR / "ci"

sys.path.insert(0, str(APPS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(CI_SCRIPTS_DIR))

import gnss_odaiba_benchmark as benchmark  # noqa: E402
import gnss_clas_ppp as clas_ppp  # noqa: E402
import gnss_live_signoff as live_signoff  # noqa: E402
import gnss_moving_base_signoff as moving_base_signoff  # noqa: E402
import gnss_ppc_commercial as ppc_commercial  # noqa: E402
import gnss_ppc_coverage_matrix as ppc_coverage_matrix  # noqa: E402
import gnss_ppc_demo as ppc_demo  # noqa: E402
import gnss_ppc_metrics as ppc_metrics  # noqa: E402
import gnss_ppc_rtk_signoff as ppc_rtk_signoff  # noqa: E402
import gnss_public_rtk_benchmarks as public_rtk_benchmarks  # noqa: E402
import gnss_smartloc_adapter as smartloc_adapter  # noqa: E402
import gnss_smartloc_signoff as smartloc_signoff  # noqa: E402
import gnss_ppp_kinematic_signoff as ppp_kinematic_signoff  # noqa: E402
import gnss_ppp_static_signoff as ppp_static_signoff  # noqa: E402
import gnss_short_baseline_signoff as short_signoff  # noqa: E402
import generate_driving_comparison as comparison  # noqa: E402
import generate_architecture_diagram as architecture_diagram  # noqa: E402
import generate_feature_overview_card as feature_overview  # noqa: E402
import generate_odaiba_scorecard as scorecard  # noqa: E402
import generate_odaiba_social_card as social_card  # noqa: E402
import analyze_ppc_coverage_quality as ppc_coverage_quality  # noqa: E402
import analyze_ppc_dual_profile_selector_matrix as ppc_dual_selector_matrix  # noqa: E402
import analyze_ppc_imu_bridge_targets as ppc_imu_bridge_targets  # noqa: E402
import analyze_ppc_imu_coverage as ppc_imu_coverage  # noqa: E402
import analyze_ppc_profile_segment_delta as ppc_profile_segment_delta  # noqa: E402
import analyze_ppc_residual_reset_sweep as ppc_residual_reset_sweep  # noqa: E402
import analyze_ppc_segment_selector_leave_one_run_out as ppc_segment_selector_loo  # noqa: E402
import analyze_ppc_segment_selector_sweep as ppc_segment_selector_sweep  # noqa: E402
import apply_ppc_dual_profile_selector as ppc_dual_profile_selector  # noqa: E402
import generate_ppc_rtk_scorecard as ppc_rtk_scorecard  # noqa: E402
import generate_ppc_selector_validation_scorecard as ppc_selector_scorecard  # noqa: E402
import generate_ppc_tail_cleanup_scorecard as ppc_tail_cleanup_scorecard  # noqa: E402
import generate_ppc_rtk_trajectory as ppc_rtk_trajectory  # noqa: E402
import run_ppc_cv_dropout_bridge_matrix as ppc_cv_bridge_matrix  # noqa: E402
import run_ppc_dual_profile_selector_matrix as ppc_dual_selector_driver  # noqa: E402
import run_ppc_imu_dropout_bridge_matrix as ppc_imu_bridge_matrix  # noqa: E402
import update_ppc_coverage_readme as ppc_coverage_readme  # noqa: E402
import detect_ci_scope as ci_scope  # noqa: E402
import run_optional_ppp_products_signoff as ci_ppp_products_signoff  # noqa: E402
import run_optional_rtk_signoffs as ci_rtk_signoffs  # noqa: E402
import analyze_ppc_multi_candidate_selector_matrix as ppc_multi_cand_analyzer  # noqa: E402
import apply_ppc_multi_candidate_selector as ppc_multi_candidate_selector  # noqa: E402
import run_ppc_multi_candidate_selector_matrix as ppc_multi_selector_matrix  # noqa: E402
import run_ppc_ratio_gating_selector_sweep as ppc_ratio_gating_sweep  # noqa: E402
import run_ppc_realtime_guard_sweep as ppc_realtime_guard_sweep  # noqa: E402


class ScorecardHelpersTest(unittest.TestCase):
    def test_ratio_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.ratio_text(0.0, 0.0), "1.0x")
        self.assertEqual(scorecard.ratio_text(5.0, 0.0), "inf")
        self.assertEqual(scorecard.ratio_text(6.0, 3.0), "2.0x")

    def test_improvement_text_handles_zero_baseline(self) -> None:
        self.assertEqual(scorecard.improvement_text(1.0, 0.0), "n/a")
        self.assertEqual(scorecard.improvement_text(1.0, 4.0), "75%")

    def test_ppc_scorecard_loads_coverage_matrix_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_scorecard_summary_") as temp_dir:
            summary_json = Path(temp_dir) / "summary.json"
            summary_json.write_text(
                json.dumps(
                    {
                        "runs": [
                            {
                                "key": "tokyo_run1",
                                "metrics": {
                                    "positioning_rate_pct": 86.2,
                                    "fix_rate_pct": 48.6,
                                    "ppc_official_score_pct": 42.0,
                                },
                                "rtklib": {
                                    "positioning_rate_pct": 66.3,
                                    "fix_rate_pct": 30.5,
                                    "ppc_official_score_pct": 21.0,
                                },
                                "delta_vs_rtklib": {
                                    "positioning_rate_pct": 19.9,
                                    "ppc_official_score_pct": 21.0,
                                    "ppc_score_3d_50cm_ref_pct": 35.6,
                                    "p95_h_m": -6.97,
                                },
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )

            runs = ppc_rtk_scorecard.runs_from_summary(summary_json)

            self.assertEqual(len(runs), 1)
            self.assertEqual(runs[0].label, "Tokyo r1")
            self.assertEqual(runs[0].positioning_delta_pct, 19.9)
            self.assertEqual(runs[0].lib_fix_pct - runs[0].rtklib_fix_pct, 18.1)
            self.assertEqual(runs[0].official_score_delta_pct, 21.0)


class ClasCompactHelpersTest(unittest.TestCase):
    def test_expand_compact_ssr_text_merges_high_rate_clock_and_system_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            payload = clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05",
                        "2200,345600.0,QZSS,3,0.0,0.0,0.0,0.1",
                    ]
                ),
                output_csv,
            )

            self.assertEqual(payload["rows_written"], 2)
            self.assertEqual(payload["systems"], ["G", "J"])
            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertIn("2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000", lines[1])
            self.assertIn("2200,345600.000,J03,0.000000,0.000000,0.000000,0.100000", lines[2])

    def test_expand_compact_ssr_text_preserves_optional_ura_code_and_phase_bias_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_meta_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,high_rate_clock_m,ura_sigma_m=<m>,cbias:<id>=<m>,pbias:<id>=<m>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,0.05,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(lines[0], "# week,tow,sat,dx,dy,dz,dclock_m")
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.450000,ura_sigma_m=0.002750,cbias:2=-0.120000,pbias:2=0.015000",
            )

    def test_expand_compact_ssr_text_preserves_atmos_metadata_tokens(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_clas_compact_atmos_") as temp_dir:
            output_csv = Path(temp_dir) / "expanded.csv"
            clas_ppp.expand_compact_ssr_text(
                "\n".join(
                    [
                        "# week,tow,system,prn,dx,dy,dz,dclock_m,atmos_<name>=<value>",
                        "2200,345600.0,G,3,0.1,0.2,0.3,0.4,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
                    ]
                ),
                output_csv,
            )

            lines = output_csv.read_text(encoding="ascii").splitlines()
            self.assertEqual(
                lines[1],
                "2200,345600.000,G03,0.100000,0.200000,0.300000,0.400000,atmos_network_id=1,atmos_trop_avail=3,atmos_stec_avail=3",
            )

    def test_parse_ppp_summary_counts_extracts_atmospheric_lines(self) -> None:
        parsed = clas_ppp._parse_ppp_summary_counts(
            "\n".join(
                [
                    "PPP summary:",
                    "  valid solutions: 4",
                    "  atmospheric trop corrections: 12",
                    "  atmospheric trop meters: 5.500000",
                    "  atmospheric ionosphere corrections: 8",
                    "  atmospheric ionosphere meters: 3.250000",
                ]
            )
        )
        self.assertEqual(parsed["ppp_atmospheric_trop_corrections"], 12)
        self.assertEqual(parsed["ppp_atmospheric_ionosphere_corrections"], 8)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_trop_meters"]), 5.5)
        self.assertAlmostEqual(float(parsed["ppp_atmospheric_ionosphere_meters"]), 3.25)


class PPCRTKSignoffHelpersTest(unittest.TestCase):
    def test_selected_thresholds_keep_rtklib_gates_only_when_enabled(self) -> None:
        args = argparse.Namespace(**{name: None for name in ppc_rtk_signoff.REQUIREMENT_NAMES})

        tokyo_without_rtklib = ppc_rtk_signoff.selected_thresholds(args, "tokyo", False)
        self.assertNotIn("require_lib_fix_rate_vs_rtklib_min_delta", tokyo_without_rtklib)
        self.assertEqual(tokyo_without_rtklib["require_fix_rate_min"], 95.0)

        nagoya_with_rtklib = ppc_rtk_signoff.selected_thresholds(args, "nagoya", True)
        self.assertIn("require_lib_fix_rate_vs_rtklib_min_delta", nagoya_with_rtklib)
        self.assertEqual(nagoya_with_rtklib["require_max_h_max"], 0.60)

    def test_build_ppc_demo_command_includes_profile_thresholds_and_rtklib_flags(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_signoff_unit_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                max_epochs=120,
                match_tolerance_s=0.25,
                city="tokyo",
                rover=None,
                base=None,
                nav=None,
                reference_csv=None,
                use_existing_solution=True,
                solver_wall_time_s=1.5,
                rtklib_bin=temp_root / "rnx2rtkp",
                rtklib_config=temp_root / "rtklib.conf",
                rtklib_pos=temp_root / "rtklib.pos",
                use_existing_rtklib_solution=True,
                rtklib_solver_wall_time_s=0.8,
                commercial_pos=temp_root / "commercial.csv",
                commercial_rover=None,
                commercial_base=None,
                commercial_nav=None,
                commercial_out=None,
                use_existing_commercial_solution=False,
                commercial_format="csv",
                commercial_label="survey_receiver",
                commercial_matched_csv=temp_root / "commercial_matches.csv",
                commercial_solver_wall_time_s=0.9,
                commercial_preset=None,
                commercial_arfilter=None,
                commercial_arfilter_margin=None,
                commercial_min_hold_count=None,
                commercial_hold_ratio_threshold=None,
                preset=None,
                iono=None,
                ratio=None,
                max_hold_div=None,
                max_pos_jump=None,
                max_pos_jump_min=None,
                max_pos_jump_rate=None,
                max_consec_float_reset=None,
                max_consec_nonfix_reset=None,
                max_postfix_rms=None,
                enable_wide_lane_ar=False,
                wide_lane_threshold=None,
                arfilter=None,
                arfilter_margin=None,
                min_hold_count=None,
                hold_ratio_threshold=None,
                no_kinematic_post_filter=True,
            )
            run_dir = temp_root / "tokyo" / "run1"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            thresholds = {
                "require_fix_rate_min": 95.0,
                "require_lib_fix_rate_vs_rtklib_min_delta": 0.0,
            }
            tuning = {
                "preset": "low-cost",
                "iono": "iflc",
                "ratio": 2.4,
                "max_hold_div": 5.0,
                "max_pos_jump": 20.0,
                "max_pos_jump_min": 20.0,
                "max_pos_jump_rate": 25.0,
                "max_float_spp_div": 30.0,
                "max_float_prefit_rms": 6.0,
                "max_float_prefit_max": 30.0,
                "max_float_prefit_reset_streak": 5,
                "min_float_prefit_trusted_jump": 8.0,
                "max_update_nis_per_obs": 12.0,
                "max_consec_float_reset": 10,
                "max_consec_nonfix_reset": 10,
                "max_postfix_rms": 0.20,
                "enable_wide_lane_ar": True,
                "wide_lane_threshold": 0.10,
                "nonfix_drift_max_anchor_gap": 90.0,
                "nonfix_drift_max_anchor_speed": 0.75,
                "nonfix_drift_max_residual": 4.0,
                "nonfix_drift_min_horizontal_residual": 6.0,
                "nonfix_drift_min_segment_epochs": 20,
                "nonfix_drift_max_segment_epochs": 180,
                "fixed_bridge_burst_guard": True,
                "fixed_bridge_burst_max_anchor_gap": 30.0,
                "fixed_bridge_burst_min_boundary_gap": 1.0,
                "fixed_bridge_burst_max_residual": 20.0,
                "fixed_bridge_burst_max_segment_epochs": 12,
                "arfilter": True,
                "arfilter_margin": 0.35,
                "min_hold_count": 8,
                "hold_ratio_threshold": 2.6,
            }

            command = ppc_rtk_signoff.build_ppc_demo_command(
                args, run_dir, out, summary_json, thresholds, tuning
            )

            self.assertEqual(command[:3], [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "ppc-demo"])
            self.assertIn("--use-existing-solution", command)
            self.assertIn("--rtklib-bin", command)
            self.assertIn(str(args.rtklib_bin), command)
            self.assertIn("--use-existing-rtklib-solution", command)
            self.assertIn("--commercial-pos", command)
            self.assertIn(str(args.commercial_pos), command)
            self.assertIn("--commercial-matched-csv", command)
            self.assertIn(str(args.commercial_matched_csv), command)
            self.assertIn("--require-fix-rate-min", command)
            self.assertIn("95.0", command)
            self.assertIn("--require-lib-fix-rate-vs-rtklib-min-delta", command)
            self.assertIn("--preset", command)
            self.assertIn("low-cost", command)
            self.assertIn("--iono", command)
            self.assertIn("iflc", command)
            self.assertIn("--ratio", command)
            self.assertIn("2.4", command)
            self.assertIn("--max-hold-div", command)
            self.assertIn("5.0", command)
            self.assertIn("--max-pos-jump", command)
            self.assertIn("20.0", command)
            self.assertIn("--max-pos-jump-min", command)
            self.assertIn("20.0", command)
            self.assertIn("--max-pos-jump-rate", command)
            self.assertIn("25.0", command)
            self.assertIn("--max-float-spp-div", command)
            self.assertIn("30.0", command)
            self.assertIn("--max-float-prefit-rms", command)
            self.assertIn("6.0", command)
            self.assertIn("--max-float-prefit-max", command)
            self.assertIn("30.0", command)
            self.assertIn("--max-float-prefit-reset-streak", command)
            self.assertIn("5", command)
            self.assertIn("--min-float-prefit-trusted-jump", command)
            self.assertIn("8.0", command)
            self.assertIn("--max-update-nis-per-obs", command)
            self.assertIn("12.0", command)
            self.assertIn("--max-consec-float-reset", command)
            self.assertIn("10", command)
            self.assertIn("--max-consec-nonfix-reset", command)
            self.assertIn("--max-postfix-rms", command)
            self.assertIn("0.2", command)
            self.assertIn("--enable-wide-lane-ar", command)
            self.assertIn("--wide-lane-threshold", command)
            self.assertIn("0.1", command)
            self.assertIn("--nonfix-drift-max-anchor-gap", command)
            self.assertIn("90.0", command)
            self.assertIn("--nonfix-drift-max-anchor-speed", command)
            self.assertIn("0.75", command)
            self.assertIn("--nonfix-drift-max-residual", command)
            self.assertIn("4.0", command)
            self.assertIn("--nonfix-drift-min-horizontal-residual", command)
            self.assertIn("6.0", command)
            self.assertIn("--nonfix-drift-min-segment-epochs", command)
            self.assertIn("--nonfix-drift-max-segment-epochs", command)
            self.assertIn("180", command)
            self.assertIn("--fixed-bridge-burst-guard", command)
            self.assertIn("--fixed-bridge-burst-max-anchor-gap", command)
            self.assertIn("30.0", command)
            self.assertIn("--fixed-bridge-burst-max-residual", command)
            self.assertIn("--fixed-bridge-burst-max-segment-epochs", command)
            self.assertIn("12", command)
            self.assertIn("--arfilter", command)
            self.assertIn("--min-hold-count", command)
            self.assertIn("8", command)
            self.assertIn("--no-kinematic-post-filter", command)

    def test_build_ppc_demo_command_passes_commercial_rover_flags(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtk_commercial_rover_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                max_epochs=120,
                match_tolerance_s=0.25,
                city="tokyo",
                rover=temp_root / "rover_ublox.obs",
                base=temp_root / "base_trimble.obs",
                nav=temp_root / "base.nav",
                reference_csv=temp_root / "reference.csv",
                use_existing_solution=True,
                solver_wall_time_s=None,
                rtklib_bin=None,
                rtklib_config=temp_root / "rtklib.conf",
                rtklib_pos=None,
                use_existing_rtklib_solution=False,
                rtklib_solver_wall_time_s=None,
                commercial_pos=None,
                commercial_rover=temp_root / "rover_trimble.obs",
                commercial_base=temp_root / "base_trimble.obs",
                commercial_nav=temp_root / "base.nav",
                commercial_out=temp_root / "commercial.pos",
                use_existing_commercial_solution=True,
                commercial_format="auto",
                commercial_label="trimble_net_r9",
                commercial_matched_csv=temp_root / "commercial_matches.csv",
                commercial_solver_wall_time_s=1.2,
                commercial_preset="survey",
                commercial_arfilter=False,
                commercial_arfilter_margin=0.2,
                commercial_min_hold_count=4,
                commercial_hold_ratio_threshold=2.0,
                preset=None,
                iono=None,
                ratio=None,
                max_hold_div=None,
                max_pos_jump=None,
                max_pos_jump_min=None,
                max_pos_jump_rate=None,
                arfilter=None,
                arfilter_margin=None,
                min_hold_count=None,
                hold_ratio_threshold=None,
            )
            command = ppc_rtk_signoff.build_ppc_demo_command(
                args,
                temp_root / "tokyo" / "run1",
                temp_root / "solution.pos",
                temp_root / "summary.json",
                {"require_fix_rate_min": 95.0},
                {"preset": "low-cost"},
            )

            self.assertIn("--commercial-rover", command)
            self.assertIn(str(args.commercial_rover), command)
            self.assertIn("--rover", command)
            self.assertIn(str(args.rover), command)
            self.assertIn("--reference-csv", command)
            self.assertIn(str(args.reference_csv), command)
            self.assertIn("--commercial-base", command)
            self.assertIn(str(args.commercial_base), command)
            self.assertIn("--commercial-nav", command)
            self.assertIn(str(args.commercial_nav), command)
            self.assertIn("--commercial-out", command)
            self.assertIn(str(args.commercial_out), command)
            self.assertIn("--use-existing-commercial-solution", command)
            self.assertIn("--commercial-preset", command)
            self.assertIn("survey", command)
            self.assertIn("--no-commercial-arfilter", command)
            self.assertIn("--commercial-min-hold-count", command)
            self.assertIn("4", command)

    def test_selected_tuning_uses_city_specific_defaults(self) -> None:
        args = argparse.Namespace(
            preset=None,
            arfilter=None,
            arfilter_margin=None,
            min_hold_count=None,
            hold_ratio_threshold=None,
        )
        tokyo = ppc_rtk_signoff.selected_tuning(args, "tokyo")
        nagoya = ppc_rtk_signoff.selected_tuning(args, "nagoya")
        self.assertEqual(tokyo["preset"], "low-cost")
        self.assertEqual(tokyo["arfilter"], True)
        self.assertEqual(nagoya["preset"], "low-cost")
        self.assertEqual(nagoya["arfilter"], False)


class PPCCoverageMatrixTest(unittest.TestCase):
    def test_validate_inputs_rejects_invalid_epoch_limit(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_coverage_validate_") as temp_dir:
            dataset_root = Path(temp_dir) / "PPC-Dataset"
            dataset_root.mkdir()
            args = argparse.Namespace(
                dataset_root=dataset_root,
                max_epochs=-2,
                rtklib_root=None,
                rtklib_bin=None,
                rtklib_config=ROOT_DIR / "scripts" / "rtklib_odaiba.conf",
            )

            with self.assertRaises(SystemExit):
                ppc_coverage_matrix.validate_inputs(args)

    def test_build_ppc_demo_command_uses_coverage_profile_and_rtklib_root(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_coverage_matrix_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                dataset_root=temp_root / "PPC-Dataset",
                output_dir=temp_root / "out",
                max_epochs=-1,
                match_tolerance_s=0.25,
                preset="low-cost",
                iono="iflc",
                ratio=2.4,
                max_hold_div=5.0,
                max_pos_jump=20.0,
                max_pos_jump_min=20.0,
                max_pos_jump_rate=25.0,
                max_float_spp_div=30.0,
                max_float_prefit_rms=6.0,
                max_float_prefit_max=30.0,
                max_float_prefit_reset_streak=5,
                min_float_prefit_trusted_jump=8.0,
                max_update_nis_per_obs=12.0,
                max_consec_float_reset=10,
                max_consec_nonfix_reset=10,
                max_postfix_rms=0.20,
                enable_wide_lane_ar=True,
                wide_lane_threshold=0.10,
                fixed_bridge_burst_guard=True,
                fixed_bridge_burst_max_anchor_gap=30.0,
                fixed_bridge_burst_min_boundary_gap=1.0,
                fixed_bridge_burst_max_residual=20.0,
                fixed_bridge_burst_max_segment_epochs=12,
                rtklib_root=temp_root / "benchmark",
                rtklib_bin=None,
                rtklib_config=ROOT_DIR / "scripts" / "rtklib_odaiba.conf",
                use_existing_solutions=False,
                no_nonfix_drift_guard=False,
                nonfix_drift_max_anchor_gap=90.0,
                nonfix_drift_max_anchor_speed=0.75,
                nonfix_drift_max_residual=4.0,
                nonfix_drift_min_horizontal_residual=6.0,
                nonfix_drift_min_segment_epochs=20,
                nonfix_drift_max_segment_epochs=180,
                no_spp_height_step_guard=False,
                spp_height_step_min=25.0,
                spp_height_step_rate=3.0,
                no_float_bridge_tail_guard=False,
                float_bridge_tail_max_anchor_gap=100.0,
                float_bridge_tail_min_anchor_speed=0.3,
                float_bridge_tail_max_anchor_speed=1.2,
                float_bridge_tail_max_residual=10.0,
                float_bridge_tail_min_segment_epochs=18,
            )
            paths = ppc_coverage_matrix.output_paths(args.output_dir, "tokyo", "run1")

            command = ppc_coverage_matrix.build_ppc_demo_command(args, "tokyo", "run1", paths)

            self.assertEqual(command[:3], [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "ppc-demo"])
            self.assertIn("--max-epochs", command)
            self.assertIn("-1", command)
            self.assertIn("--no-arfilter", command)
            self.assertIn("--no-kinematic-post-filter", command)
            self.assertIn("--iono", command)
            self.assertIn("iflc", command)
            self.assertIn("--ratio", command)
            self.assertIn("2.4", command)
            self.assertIn("--max-hold-div", command)
            self.assertIn("5.0", command)
            self.assertIn("--max-pos-jump", command)
            self.assertIn("20.0", command)
            self.assertIn("--max-pos-jump-min", command)
            self.assertIn("20.0", command)
            self.assertIn("--max-pos-jump-rate", command)
            self.assertIn("25.0", command)
            self.assertIn("--max-float-spp-div", command)
            self.assertIn("30.0", command)
            self.assertIn("--max-float-prefit-rms", command)
            self.assertIn("6.0", command)
            self.assertIn("--max-float-prefit-max", command)
            self.assertIn("30.0", command)
            self.assertIn("--max-float-prefit-reset-streak", command)
            self.assertIn("5", command)
            self.assertIn("--min-float-prefit-trusted-jump", command)
            self.assertIn("8.0", command)
            self.assertIn("--max-update-nis-per-obs", command)
            self.assertIn("12.0", command)
            self.assertIn("--max-consec-float-reset", command)
            self.assertIn("10", command)
            self.assertIn("--max-consec-nonfix-reset", command)
            self.assertIn("--max-postfix-rms", command)
            self.assertIn("0.2", command)
            self.assertIn("--enable-wide-lane-ar", command)
            self.assertIn("--wide-lane-threshold", command)
            self.assertIn("0.1", command)
            self.assertIn("--nonfix-drift-max-anchor-gap", command)
            self.assertIn("90.0", command)
            self.assertIn("--nonfix-drift-max-anchor-speed", command)
            self.assertIn("0.75", command)
            self.assertIn("--nonfix-drift-max-residual", command)
            self.assertIn("4.0", command)
            self.assertIn("--nonfix-drift-min-horizontal-residual", command)
            self.assertIn("6.0", command)
            self.assertIn("--nonfix-drift-min-segment-epochs", command)
            self.assertIn("20", command)
            self.assertIn("--nonfix-drift-max-segment-epochs", command)
            self.assertIn("180", command)
            self.assertIn("--spp-height-step-min", command)
            self.assertIn("25.0", command)
            self.assertIn("--spp-height-step-rate", command)
            self.assertIn("3.0", command)
            self.assertIn("--float-bridge-tail-max-anchor-gap", command)
            self.assertIn("100.0", command)
            self.assertIn("--float-bridge-tail-min-anchor-speed", command)
            self.assertIn("0.3", command)
            self.assertIn("--float-bridge-tail-max-anchor-speed", command)
            self.assertIn("1.2", command)
            self.assertIn("--float-bridge-tail-max-residual", command)
            self.assertIn("10.0", command)
            self.assertIn("--float-bridge-tail-min-segment-epochs", command)
            self.assertIn("18", command)
            self.assertIn("--fixed-bridge-burst-guard", command)
            self.assertIn("--fixed-bridge-burst-max-residual", command)
            self.assertIn("20.0", command)
            self.assertIn("--rtklib-pos", command)
            self.assertIn(str(temp_root / "benchmark" / "tokyo_run1" / "rtklib.pos"), command)
            self.assertIn("--use-existing-rtklib-solution", command)
            self.assertNotIn("--no-float-bridge-tail-guard", command)

            args.no_float_bridge_tail_guard = True
            disabled_command = ppc_coverage_matrix.build_ppc_demo_command(args, "tokyo", "run1", paths)
            self.assertIn("--no-float-bridge-tail-guard", disabled_command)

    def test_matrix_payload_aggregates_rtklib_deltas_and_guard_counts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_coverage_payload_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                dataset_root=temp_root / "PPC-Dataset",
                output_dir=temp_root / "out",
                max_epochs=-1,
                match_tolerance_s=0.25,
                preset="low-cost",
                iono=None,
                ratio=None,
                max_hold_div=None,
                max_pos_jump=None,
                max_pos_jump_min=None,
                max_pos_jump_rate=None,
                max_consec_float_reset=None,
                max_consec_nonfix_reset=None,
                max_postfix_rms=None,
                enable_wide_lane_ar=False,
                wide_lane_threshold=None,
                no_float_bridge_tail_guard=False,
            )
            paths = ppc_coverage_matrix.output_paths(args.output_dir, "tokyo", "run1")
            paths["summary"].parent.mkdir(parents=True)
            paths["summary"].write_text(
                json.dumps(
                    {
                        "positioning_rate_pct": 86.2,
                        "fix_rate_pct": 48.6,
                        "ppc_official_score_pct": 42.0,
                        "ppc_official_score_distance_m": 420.0,
                        "ppc_official_total_distance_m": 1000.0,
                        "ppc_score_3d_50cm_ref_pct": 35.6,
                        "p95_h_m": 24.16,
                        "max_h_m": 47.9,
                        "solver_wall_time_s": 1.0,
                        "realtime_factor": 10.0,
                        "effective_epoch_rate_hz": 50.0,
                        "rtklib": {
                            "positioning_rate_pct": 66.3,
                            "fix_rate_pct": 30.5,
                            "ppc_official_score_pct": 21.0,
                            "ppc_official_score_distance_m": 210.0,
                            "ppc_official_total_distance_m": 1000.0,
                        },
                        "delta_vs_rtklib": {
                            "positioning_rate_pct": 19.9,
                            "fix_rate_pct": 18.1,
                            "ppc_official_score_pct": 21.0,
                            "ppc_score_3d_50cm_ref_pct": 35.6,
                            "p95_h_m": -6.97,
                        },
                    }
                ),
                encoding="utf-8",
            )
            log_text = "\n".join(
                [
                    "  non-FIX drift guard: enabled inspected_segments=5 rejected_segments=2 rejected_epochs=320",
                    "  SPP height-step guard: enabled rejected_epochs=30",
                    "  FLOAT bridge-tail guard: enabled inspected_segments=2 rejected_segments=1 rejected_epochs=147",
                    "  fixed bridge-burst guard: enabled inspected_segments=21 rejected_segments=3 rejected_epochs=12",
                ]
            )
            run = ppc_coverage_matrix.load_run_record(
                "tokyo",
                "run1",
                paths,
                ["gnss", "ppc-demo"],
                log_text,
                2.5,
            )
            payload = ppc_coverage_matrix.build_matrix_payload(args, [run])
            markdown = ppc_coverage_matrix.render_markdown(payload)

            self.assertEqual(payload["aggregates"]["avg_positioning_delta_pct"], 19.9)
            self.assertEqual(payload["aggregates"]["avg_official_score_delta_pct"], 21.0)
            self.assertEqual(payload["aggregates"]["weighted_official_score_pct"], 42.0)
            self.assertEqual(payload["aggregates"]["weighted_rtklib_official_score_pct"], 21.0)
            self.assertEqual(payload["aggregates"]["weighted_official_score_delta_pct"], 21.0)
            self.assertEqual(payload["aggregates"]["avg_p95_h_delta_m"], -6.97)
            self.assertEqual(payload["aggregates"]["avg_solver_wall_time_s"], 1.0)
            self.assertEqual(payload["aggregates"]["max_solver_wall_time_s"], 1.0)
            self.assertEqual(payload["aggregates"]["avg_realtime_factor"], 10.0)
            self.assertEqual(payload["aggregates"]["min_realtime_factor"], 10.0)
            self.assertEqual(payload["aggregates"]["avg_effective_epoch_rate_hz"], 50.0)
            self.assertEqual(payload["aggregates"]["min_effective_epoch_rate_hz"], 50.0)
            self.assertEqual(payload["aggregates"]["float_bridge_tail_rejected_epochs"], 147)
            self.assertEqual(payload["aggregates"]["fixed_bridge_burst_rejected_epochs"], 12)
            self.assertEqual(payload["runtime_requirements"]["solver_wall_time_max_s"], None)
            self.assertEqual(payload["runtime_requirements"]["realtime_factor_min"], None)
            self.assertEqual(payload["runtime_requirements"]["effective_epoch_rate_min_hz"], None)
            self.assertIsNone(payload["max_pos_jump_min"])
            self.assertIsNone(payload["max_pos_jump_rate"])
            self.assertIsNone(payload["max_float_prefit_rms"])
            self.assertIsNone(payload["max_float_prefit_max"])
            self.assertIsNone(payload["max_float_prefit_reset_streak"])
            self.assertIsNone(payload["min_float_prefit_trusted_jump"])
            self.assertIsNone(payload["max_update_nis_per_obs"])
            self.assertIsNone(payload["max_consec_float_reset"])
            self.assertIsNone(payload["max_consec_nonfix_reset"])
            self.assertIsNone(payload["max_postfix_rms"])
            self.assertFalse(payload["enable_wide_lane_ar"])
            self.assertIsNone(payload["wide_lane_threshold"])
            self.assertIn("tokyo_run1", markdown)
            self.assertIn("+19.9 pp", markdown)
            self.assertIn("42.0%", markdown)
            self.assertIn("PPC official weighted delta: 21.0 pp", markdown)
            self.assertIn("Realtime factor: avg 10.0, min 10.0", markdown)
            self.assertIn("50.00 Hz", markdown)
            self.assertIn("147", markdown)
            self.assertIn("12", markdown)

            ppc_coverage_matrix.enforce_requirements(
                payload,
                argparse.Namespace(
                    require_positioning_delta_min=0.0,
                    require_fix_delta_min=0.0,
                    require_official_score_delta_min=0.0,
                    require_score_3d_50cm_ref_delta_min=0.0,
                    require_p95_h_delta_max=0.0,
                    require_solver_wall_time_max=1.0,
                    require_realtime_factor_min=10.0,
                    require_effective_epoch_rate_min=50.0,
                ),
            )
            with self.assertRaises(SystemExit):
                ppc_coverage_matrix.enforce_requirements(
                    payload,
                    argparse.Namespace(
                        require_positioning_delta_min=20.0,
                        require_fix_delta_min=None,
                        require_official_score_delta_min=None,
                        require_score_3d_50cm_ref_delta_min=None,
                        require_p95_h_delta_max=None,
                        require_solver_wall_time_max=None,
                        require_realtime_factor_min=None,
                        require_effective_epoch_rate_min=None,
                    ),
                )
            with self.assertRaises(SystemExit) as runtime_context:
                ppc_coverage_matrix.enforce_requirements(
                    payload,
                    argparse.Namespace(
                        require_positioning_delta_min=None,
                        require_fix_delta_min=None,
                        require_official_score_delta_min=None,
                        require_score_3d_50cm_ref_delta_min=None,
                        require_p95_h_delta_max=None,
                        require_solver_wall_time_max=0.5,
                        require_realtime_factor_min=20.0,
                        require_effective_epoch_rate_min=100.0,
                    ),
                )
            runtime_message = str(runtime_context.exception)
            self.assertIn("solver_wall_time_s", runtime_message)
            self.assertIn("realtime_factor", runtime_message)
            self.assertIn("effective_epoch_rate_hz", runtime_message)


class PPCResidualResetSweepAnalysisTest(unittest.TestCase):
    @staticmethod
    def summary_run(
        key: str,
        score_m: float,
        total_m: float,
        positioning_pct: float,
        p95_h_m: float,
    ) -> dict[str, object]:
        city, _, run_name = key.partition("_")
        return {
            "key": key,
            "city": city,
            "run": run_name,
            "metrics": {
                "ppc_official_score_distance_m": score_m,
                "ppc_official_total_distance_m": total_m,
                "positioning_rate_pct": positioning_pct,
                "p95_h_m": p95_h_m,
            },
        }

    @staticmethod
    def write_summary(path: Path, runs: list[dict[str, object]]) -> Path:
        path.write_text(json.dumps({"runs": runs}), encoding="utf-8")
        return path

    def test_selector_analysis_reports_global_city_and_run_oracles(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_residual_reset_sweep_") as temp_dir:
            temp_root = Path(temp_dir)
            baseline_path = self.write_summary(
                temp_root / "baseline.json",
                [
                    self.summary_run("tokyo_run1", 500.0, 1000.0, 90.0, 4.0),
                    self.summary_run("tokyo_run2", 700.0, 1000.0, 92.0, 5.0),
                    self.summary_run("nagoya_run1", 600.0, 2000.0, 80.0, 8.0),
                ],
            )
            candidate_a_path = self.write_summary(
                temp_root / "candidate_a.json",
                [
                    self.summary_run("tokyo_run1", 540.0, 1000.0, 91.0, 3.5),
                    self.summary_run("tokyo_run2", 690.0, 1000.0, 91.5, 5.5),
                    self.summary_run("nagoya_run1", 580.0, 2000.0, 78.0, 8.5),
                ],
            )
            candidate_b_path = self.write_summary(
                temp_root / "candidate_b.json",
                [
                    self.summary_run("tokyo_run1", 520.0, 1000.0, 89.0, 3.0),
                    self.summary_run("tokyo_run2", 740.0, 1000.0, 92.5, 4.5),
                    self.summary_run("nagoya_run1", 590.0, 2000.0, 79.0, 8.2),
                ],
            )

            baseline = ppc_residual_reset_sweep.load_profile("baseline", baseline_path)
            candidate_a = ppc_residual_reset_sweep.load_profile("candidate_a", candidate_a_path)
            candidate_b = ppc_residual_reset_sweep.load_profile("candidate_b", candidate_b_path)
            payload = ppc_residual_reset_sweep.build_payload(baseline, [candidate_a, candidate_b])
            markdown = ppc_residual_reset_sweep.render_markdown(payload)

            self.assertEqual(payload["profiles"][0]["weighted_official_score_pct"], 45.0)
            self.assertEqual(payload["global_best_profile"]["label"], "candidate_b")
            self.assertEqual(payload["global_best_profile"]["weighted_official_score_pct"], 46.25)
            self.assertEqual(payload["best_by_city_selector"]["weighted_official_score_pct"], 46.5)
            self.assertEqual(payload["best_by_city_selector"]["delta_vs_baseline_score_distance_m"], 60.0)
            city_profiles = {
                row["city"]: row["profile"]
                for row in payload["best_by_city_selector"]["selections"]
            }
            self.assertEqual(city_profiles["tokyo"], "candidate_b")
            self.assertEqual(city_profiles["nagoya"], "baseline")
            self.assertEqual(payload["best_per_run_oracle"]["weighted_official_score_pct"], 47.0)
            run_profiles = {row["key"]: row["best_profile"] for row in payload["runs"]}
            self.assertEqual(run_profiles["tokyo_run1"], "candidate_a")
            self.assertEqual(run_profiles["tokyo_run2"], "candidate_b")
            self.assertEqual(run_profiles["nagoya_run1"], "baseline")
            self.assertIn("Global best profile: **candidate_b**", markdown)
            self.assertIn("Per-run oracle: **47.00%**", markdown)

            summary_json = temp_root / "analysis.json"
            markdown_output = temp_root / "analysis.md"
            ppc_residual_reset_sweep.write_outputs(
                payload,
                argparse.Namespace(summary_json=summary_json, markdown_output=markdown_output),
            )
            written = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(
                written["best_per_run_oracle"]["weighted_official_score_pct"],
                47.0,
            )
            self.assertIn("City Selector", markdown_output.read_text(encoding="utf-8"))


class PPCRealtimeGuardSweepTest(unittest.TestCase):
    def test_profile_command_passes_runtime_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_realtime_guard_") as temp_dir:
            temp_root = Path(temp_dir)
            args = argparse.Namespace(
                dataset_root=temp_root / "PPC-Dataset",
                output_dir=temp_root / "out",
                summary_json=temp_root / "summary.json",
                markdown_output=temp_root / "summary.md",
                max_epochs=-1,
                match_tolerance_s=0.25,
                preset="low-cost",
                rtklib_root=temp_root / "rtklib",
                rtklib_bin=None,
                rtklib_config=ROOT_DIR / "scripts" / "rtklib_odaiba.conf",
                use_existing_solutions=True,
                require_positioning_delta_min=0.0,
                require_official_score_delta_min=0.0,
                require_p95_h_delta_max=0.0,
                require_solver_wall_time_max=10.0,
                require_realtime_factor_min=1.0,
                require_effective_epoch_rate_min=5.0,
            )
            profile = ppc_realtime_guard_sweep.parse_profile_spec(
                "fixed=--ratio 2.4 --max-fixed-update-nis-per-obs 10"
            )

            command, matrix_json, matrix_md = ppc_realtime_guard_sweep.build_matrix_argv(
                args,
                profile,
            )

            self.assertEqual(command[:3], [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "ppc-coverage-matrix"])
            self.assertEqual(matrix_json, temp_root / "out" / "fixed" / "matrix.json")
            self.assertEqual(matrix_md, temp_root / "out" / "fixed" / "matrix.md")
            self.assertIn("--use-existing-solutions", command)
            self.assertIn("--rtklib-root", command)
            self.assertIn(str(temp_root / "rtklib"), command)
            self.assertIn("--require-solver-wall-time-max", command)
            self.assertIn("10.0", command)
            self.assertIn("--require-realtime-factor-min", command)
            self.assertIn("1.0", command)
            self.assertIn("--require-effective-epoch-rate-min", command)
            self.assertIn("5.0", command)
            self.assertIn("--max-fixed-update-nis-per-obs", command)
            self.assertIn("10", command)

    def test_payload_ranks_profiles_with_runtime_metrics(self) -> None:
        baseline = ppc_realtime_guard_sweep.GuardProfile("coverage", ("--ratio", "2.4"))
        candidate = ppc_realtime_guard_sweep.GuardProfile(
            "fixed_update_guard",
            (
                "--ratio",
                "2.4",
                "--max-fixed-update-nis-per-obs",
                "10",
            ),
        )
        baseline_payload = {
            "runs": [
                {
                    "key": "tokyo_run1",
                    "metrics": {
                        "ppc_official_score_distance_m": 400.0,
                        "ppc_official_total_distance_m": 1000.0,
                    },
                },
            ],
            "aggregates": {
                "avg_positioning_delta_pct": 17.0,
                "avg_p95_h_delta_m": -10.0,
                "max_solver_wall_time_s": 2.0,
                "min_realtime_factor": 4.0,
                "min_effective_epoch_rate_hz": 20.0,
            },
        }
        candidate_payload = {
            "runs": [
                {
                    "key": "tokyo_run1",
                    "metrics": {
                        "ppc_official_score_distance_m": 430.0,
                        "ppc_official_total_distance_m": 1000.0,
                    },
                },
            ],
            "aggregates": {
                "avg_positioning_delta_pct": 16.5,
                "avg_p95_h_delta_m": -11.0,
                "max_solver_wall_time_s": 2.5,
                "min_realtime_factor": 3.5,
                "min_effective_epoch_rate_hz": 18.0,
            },
        }

        payload = ppc_realtime_guard_sweep.build_payload(
            [
                (baseline, Path("coverage.json"), Path("coverage.md"), baseline_payload),
                (candidate, Path("fixed.json"), Path("fixed.md"), candidate_payload),
            ]
        )
        markdown = ppc_realtime_guard_sweep.render_markdown(payload)

        self.assertEqual(payload["baseline_profile"], "coverage")
        self.assertEqual(payload["best_profile"]["name"], "fixed_update_guard")
        self.assertEqual(payload["profiles"][1]["delta_vs_baseline_score_pct"], 3.0)
        self.assertEqual(payload["profiles"][1]["min_realtime_factor"], 3.5)
        self.assertIn("fixed_update_guard", markdown)
        self.assertIn("3.500", markdown)
        self.assertIn("--max-fixed-update-nis-per-obs", markdown)


class PPCProfileSegmentDeltaTest(unittest.TestCase):
    @staticmethod
    def record(
        reference_index: int,
        distance_m: float,
        score_state: str,
        status: int | None,
        error_3d_m: float | None,
        ratio: float | None = None,
        prefit_rms_m: float | None = None,
    ) -> dict[str, object]:
        return {
            "reference_index": reference_index,
            "start_tow_s": float(reference_index - 1),
            "end_tow_s": float(reference_index),
            "segment_distance_m": distance_m,
            "matched": status is not None,
            "scored": score_state == "scored",
            "score_state": score_state,
            "score_threshold_m": 0.5,
            "score_distance_m": distance_m if score_state == "scored" else 0.0,
            "matched_distance_m": distance_m if status is not None else 0.0,
            "solution_tow_s": float(reference_index) if status is not None else None,
            "time_gap_s": 0.0 if status is not None else None,
            "status": status,
            "num_satellites": 12 if status is not None else None,
            "ratio": ratio,
            "baseline_m": 100.0 if status is not None else None,
            "rtk_iterations": 2 if status is not None else None,
            "rtk_update_observations": 16 if status is not None else None,
            "rtk_update_phase_observations": 8 if status is not None else None,
            "rtk_update_code_observations": 8 if status is not None else None,
            "rtk_update_suppressed_outliers": 1 if status is not None else None,
            "rtk_update_prefit_residual_rms_m": prefit_rms_m,
            "rtk_update_prefit_residual_max_m": None if prefit_rms_m is None else prefit_rms_m * 4.0,
            "rtk_update_post_suppression_residual_rms_m": None,
            "rtk_update_post_suppression_residual_max_m": None,
            "rtk_update_normalized_innovation_squared": (
                None if prefit_rms_m is None else prefit_rms_m * 160.0
            ),
            "rtk_update_normalized_innovation_squared_per_observation": (
                None if prefit_rms_m is None else prefit_rms_m * 10.0
            ),
            "rtk_update_rejected_by_innovation_gate": 0 if status is not None else None,
            "error_3d_m": error_3d_m,
            "horiz_error_m": error_3d_m,
            "up_error_m": 0.0 if error_3d_m is not None else None,
        }

    def test_segment_delta_reports_gain_loss_transitions_and_csv(self) -> None:
        baseline = [
            self.record(1, 10.0, "scored", 4, 0.2, 20.0, 0.2),
            self.record(2, 20.0, "high_error", 3, 1.2, 4.0, 6.0),
            self.record(3, 30.0, "scored", 4, 0.1, 18.0, 0.1),
        ]
        candidate = [
            self.record(1, 10.0, "high_error", 3, 0.8, 3.0, 7.0),
            self.record(2, 20.0, "scored", 4, 0.2, 24.0, 0.3),
            self.record(3, 30.0, "scored", 4, 0.1, 18.0, 0.1),
        ]

        summary = ppc_profile_segment_delta.compare_segment_records(
            baseline,
            candidate,
            "candidate",
            top_segments=4,
        )

        self.assertEqual(summary["delta_vs_baseline_score_distance_m"], 10.0)
        self.assertEqual(summary["gain_distance_m"], 20.0)
        self.assertEqual(summary["loss_distance_m"], -10.0)
        self.assertEqual(summary["changed_segments"], 2)
        transitions = {
            row["score_transition"]: row
            for row in summary["score_transitions"]
        }
        self.assertEqual(transitions["high_error->scored"]["score_delta_distance_m"], 20.0)
        self.assertEqual(transitions["scored->high_error"]["score_delta_distance_m"], -10.0)
        self.assertEqual(summary["top_gains"][0]["reference_index"], 2)
        self.assertEqual(summary["top_losses"][0]["reference_index"], 1)
        self.assertEqual(
            summary["candidate_gain_diagnostics"]["median_rtk_update_prefit_residual_rms_m"],
            0.3,
        )
        self.assertEqual(
            summary["candidate_gain_diagnostics"][
                "median_rtk_update_normalized_innovation_squared_per_observation"
            ],
            3.0,
        )

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_segment_delta_") as temp_dir:
            temp_root = Path(temp_dir)
            csv_path = temp_root / "segments.csv"
            ppc_profile_segment_delta.write_segments_csv(csv_path, [summary])
            with csv_path.open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["candidate_label"], "candidate")
            self.assertEqual(rows[0]["score_transition"], "scored->high_error")

            markdown = ppc_profile_segment_delta.render_markdown(
                "baseline",
                temp_root / "baseline.pos",
                temp_root / "reference.csv",
                [summary],
            )
            self.assertIn("PPC Profile Segment Delta", markdown)
            self.assertIn("high_error->scored", markdown)
            self.assertIn("Top Losses", markdown)


class PPCSegmentSelectorSweepTest(unittest.TestCase):
    @staticmethod
    def selector_row(
        run_label: str,
        delta_m: float,
        distance_m: float,
        baseline_status: str,
        candidate_status: str,
        candidate_rms_m: float,
        candidate_ratio: float = 10.0,
        candidate_num_satellites: float = 12.0,
        candidate_baseline_m: float = 1000.0,
    ) -> dict[str, object]:
        return {
            "run_label": run_label,
            "segment_distance_m": distance_m,
            "score_delta_distance_m": delta_m,
            "status_transition": f"{baseline_status}->{candidate_status}",
            "baseline_status_name": baseline_status,
            "candidate_status_name": candidate_status,
            "candidate_ratio": candidate_ratio,
            "candidate_num_satellites": candidate_num_satellites,
            "candidate_baseline_m": candidate_baseline_m,
            "baseline_baseline_m": candidate_baseline_m,
            "candidate_rtk_update_observations": 16.0,
            "candidate_rtk_update_suppressed_outliers": 0.0,
            "candidate_rtk_update_prefit_residual_rms_m": candidate_rms_m,
            "candidate_rtk_update_prefit_residual_max_m": candidate_rms_m * 4.0,
            "candidate_rtk_update_post_suppression_residual_rms_m": candidate_rms_m,
            "candidate_rtk_update_post_suppression_residual_max_m": candidate_rms_m * 4.0,
            "candidate_rtk_update_normalized_innovation_squared": candidate_rms_m * 160.0,
            "candidate_rtk_update_normalized_innovation_squared_per_observation": (
                candidate_rms_m * 10.0
            ),
            "candidate_rtk_update_rejected_by_innovation_gate": 0.0,
            "baseline_ratio": 0.0,
            "baseline_num_satellites": 12.0,
        }

    def test_selector_sweep_ranks_segment_local_candidate_rules(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 12.0, 12.0, "FLOAT", "FIXED", 0.4, 20.0),
            self.selector_row("tokyo_run1", -7.0, 7.0, "FIXED", "FIXED", 5.0, 30.0),
            self.selector_row("tokyo_run2", -20.0, 20.0, "FLOAT", "FLOAT", 0.5, 0.0),
        ]
        rule = ppc_segment_selector_sweep.RuleSpec(
            categorical=(
                ppc_segment_selector_sweep.CategoricalCondition(
                    "candidate_status_name",
                    "FIXED",
                ),
            ),
            numeric=(
                ppc_segment_selector_sweep.NumericCondition(
                    "candidate_rtk_update_post_suppression_residual_rms_m",
                    "<=",
                    1.0,
                ),
            ),
        )

        score = ppc_segment_selector_sweep.score_rule(rows, rule)

        self.assertEqual(score["selected_score_delta_distance_m"], 12.0)
        self.assertEqual(score["selected_gain_distance_m"], 12.0)
        self.assertEqual(score["selected_loss_distance_m"], 0.0)
        self.assertEqual(score["avoided_loss_distance_m"], 27.0)
        self.assertEqual(score["gain_recall_pct"], 100.0)
        self.assertEqual(score["loss_exposure_pct"], 0.0)
        self.assertEqual(score["negative_run_count"], 0)
        self.assertEqual(score["min_run_score_delta_distance_m"], 0.0)

        payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=8,
            max_thresholds=16,
        )
        self.assertEqual(payload["candidate_all"]["selected_score_delta_distance_m"], -15.0)
        self.assertGreaterEqual(
            payload["top_rules"][0]["selected_score_delta_distance_m"],
            12.0,
        )
        markdown = ppc_segment_selector_sweep.render_markdown(payload)
        self.assertIn("PPC Segment Selector Sweep", markdown)
        self.assertIn("Best Rule By Run", markdown)

    def test_selector_sweep_can_require_candidate_status(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 10.0, 10.0, "FLOAT", "FIXED", 0.4, 8.0, 12.0),
            self.selector_row("tokyo_run1", -5.0, 5.0, "FLOAT", "FLOAT", 0.4, 8.0, 12.0),
        ]

        payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=4,
            max_thresholds=8,
            required_candidate_status="FIXED",
        )

        best_rule = payload["top_rules"][0]["rule"]
        self.assertEqual(payload["required_candidate_status"], "FIXED")
        self.assertTrue("candidate_status_name == FIXED" in best_rule or "->FIXED" in best_rule)

    def test_selector_sweep_robust_objective_prefers_nonnegative_runs(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 20.0, 20.0, "FIXED", "FIXED", 0.4, 8.0, 12.0, 500.0),
            self.selector_row("tokyo_run2", -5.0, 5.0, "FIXED", "FIXED", 0.4, 8.0, 12.0, 500.0),
            self.selector_row("tokyo_run1", 10.0, 10.0, "FIXED", "FIXED", 0.4, 8.0, 6.0, 1000.0),
            self.selector_row("tokyo_run2", 4.0, 4.0, "FIXED", "FIXED", 0.4, 8.0, 6.0, 1000.0),
            self.selector_row("tokyo_run2", -30.0, 30.0, "FIXED", "FIXED", 0.4, 8.0, 6.0, 500.0),
        ]

        net_payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=4,
            max_thresholds=8,
            max_numeric_conditions=1,
            rank_objective="net",
            min_selected_distance_m=1.0,
        )
        robust_payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=4,
            max_thresholds=8,
            max_numeric_conditions=1,
            rank_objective="robust",
            min_selected_distance_m=1.0,
        )

        net_best = net_payload["top_rules"][0]
        robust_best = robust_payload["top_rules"][0]
        self.assertIn("candidate_num_satellites >= 12", net_best["rule"])
        self.assertEqual(net_best["selected_score_delta_distance_m"], 15.0)
        self.assertEqual(net_best["negative_run_count"], 1)
        self.assertIn("candidate_baseline_m >= 1000", robust_best["rule"])
        self.assertEqual(robust_best["selected_score_delta_distance_m"], 14.0)
        self.assertEqual(robust_best["negative_run_count"], 0)
        self.assertEqual(robust_payload["rank_objective"], "robust")

    def test_selector_sweep_can_refine_with_two_numeric_conditions(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 12.0, 12.0, "FLOAT", "FIXED", 0.4, 8.0, 12.0),
            self.selector_row("tokyo_run1", -10.0, 10.0, "FIXED", "FIXED", 0.4, 8.0, 6.0),
            self.selector_row("tokyo_run1", -7.0, 7.0, "FIXED", "FIXED", 5.0, 8.0, 12.0),
            self.selector_row("tokyo_run2", -20.0, 20.0, "FLOAT", "FLOAT", 0.5, 0.0, 12.0),
        ]

        payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=8,
            max_thresholds=16,
            max_numeric_conditions=2,
        )

        best_rule = payload["top_rules"][0]
        self.assertEqual(best_rule["selected_score_delta_distance_m"], 12.0)
        self.assertIn("candidate_num_satellites >= 12", best_rule["rule"])
        self.assertIn("residual_rms_m <= 0.4", best_rule["rule"])

    def test_selector_sweep_refines_coarse_numeric_thresholds(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 5.0, 5.0, "FIXED", "FIXED", 0.1, 8.0),
            self.selector_row("tokyo_run1", 20.0, 20.0, "FIXED", "FIXED", 0.8, 8.0),
            self.selector_row("tokyo_run1", -15.0, 15.0, "FIXED", "FIXED", 1.5, 8.0),
        ]

        payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=8,
            max_thresholds=2,
            numeric_threshold_refinement_beam=32,
        )

        best_rule = payload["top_rules"][0]
        self.assertEqual(best_rule["selected_score_delta_distance_m"], 25.0)
        self.assertIn("residual_rms_m <= 0.8", best_rule["rule"])

    def test_selector_sweep_can_refine_with_three_numeric_conditions(self) -> None:
        rows = [
            self.selector_row("tokyo_run1", 30.0, 30.0, "FIXED", "FIXED", 0.4, 8.0, 12.0, 1000.0),
            self.selector_row("tokyo_run1", -10.0, 10.0, "FIXED", "FIXED", 0.4, 8.0, 6.0, 1000.0),
            self.selector_row("tokyo_run1", -8.0, 8.0, "FIXED", "FIXED", 5.0, 8.0, 12.0, 1000.0),
            self.selector_row("tokyo_run1", -7.0, 7.0, "FIXED", "FIXED", 0.4, 8.0, 12.0, 100.0),
        ]

        payload = ppc_segment_selector_sweep.build_payload(
            rows,
            top_rules=8,
            max_thresholds=16,
            max_numeric_conditions=3,
        )

        best_rule = payload["top_rules"][0]
        self.assertEqual(best_rule["selected_score_delta_distance_m"], 30.0)
        self.assertIn("candidate_baseline_m", best_rule["rule"])
        self.assertIn("candidate_num_satellites", best_rule["rule"])
        self.assertIn("residual_rms_m", best_rule["rule"])


class PPCSegmentSelectorLeaveOneRunOutTest(unittest.TestCase):
    def test_leave_one_run_out_scores_learned_rules_on_holdout_runs(self) -> None:
        row = PPCSegmentSelectorSweepTest.selector_row
        rows = [
            row("tokyo_run1", 10.0, 10.0, "FIXED", "FIXED", 0.4, 8.0, 12.0),
            row("tokyo_run1", -5.0, 5.0, "FIXED", "FIXED", 0.4, 8.0, 6.0),
            row("tokyo_run2", 10.0, 10.0, "FIXED", "FIXED", 0.4, 8.0, 12.0),
            row("tokyo_run2", -5.0, 5.0, "FIXED", "FIXED", 0.4, 8.0, 6.0),
        ]

        payload = ppc_segment_selector_loo.build_payload(
            rows,
            top_rules=4,
            max_thresholds=8,
            max_numeric_conditions=2,
            numeric_refinement_beam=4,
            numeric_threshold_refinement_beam=4,
            rank_objective="robust",
        )

        aggregates = payload["aggregates"]
        self.assertEqual(aggregates["fold_count"], 2)
        self.assertEqual(aggregates["holdout_selected_score_delta_distance_m"], 20.0)
        self.assertEqual(aggregates["holdout_candidate_all_delta_distance_m"], 10.0)
        self.assertEqual(aggregates["holdout_selector_vs_candidate_all_delta_m"], 10.0)
        self.assertEqual(aggregates["nonnegative_holdout_runs"], 2)
        self.assertEqual(payload["rank_objective"], "robust")
        self.assertIn("candidate_num_satellites", payload["folds"][0]["learned_rule"])

        markdown = ppc_segment_selector_loo.render_markdown(payload)
        self.assertIn("PPC Segment Selector Leave-One-Run-Out", markdown)
        self.assertIn("tokyo_run1", markdown)


class PPCDualProfileSelectorTest(unittest.TestCase):
    @staticmethod
    def reference_epoch(index: int) -> comparison.ReferenceEpoch:
        return comparison.ReferenceEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([10.0 * index, 0.0, 0.0]),
        )

    @staticmethod
    def solution_epoch(
        index: int,
        ecef_x_m: float,
        status: int,
        post_rms_m: float | None,
    ) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([ecef_x_m, 0.0, 0.0]),
            status,
            12,
            10.0 if status == 4 else 0.0,
            100.0,
            2,
            16,
            8,
            8,
            0,
            post_rms_m,
            None if post_rms_m is None else post_rms_m * 4.0,
            post_rms_m,
            None if post_rms_m is None else post_rms_m * 4.0,
        )

    def test_dual_profile_selector_writes_selected_pos_and_metrics(self) -> None:
        reference = [self.reference_epoch(index) for index in range(3)]
        baseline = [
            self.solution_epoch(0, 0.0, 4, None),
            self.solution_epoch(1, 11.2, 3, None),
            self.solution_epoch(2, 20.1, 4, None),
        ]
        candidate = [
            self.solution_epoch(0, 0.0, 4, 0.2),
            self.solution_epoch(1, 10.1, 4, 0.3),
            self.solution_epoch(2, 21.0, 3, 0.2),
        ]
        baseline_records = ppc_metrics.ppc_official_segment_records(reference, baseline, 0.25)
        candidate_records = ppc_metrics.ppc_official_segment_records(reference, candidate, 0.25)
        rows = ppc_dual_profile_selector.all_segment_rows(
            baseline_records,
            candidate_records,
            "candidate",
        )
        ppc_dual_profile_selector.augment_solution_tows(
            rows,
            baseline_records,
            candidate_records,
        )
        rule = ppc_dual_profile_selector.parse_rule(
            "candidate_status_name == FIXED AND "
            "candidate_rtk_update_post_suppression_residual_rms_m <= 1.0"
        )
        selected, selected_rows = ppc_dual_profile_selector.selected_solution_epochs(
            reference,
            baseline,
            candidate,
            rows,
            rule,
            0.25,
        )

        self.assertEqual([epoch.tow for epoch in selected], [0.0, 1.0, 2.0])
        self.assertEqual(selected[1].status, 4)
        self.assertEqual(selected[2].ecef[0], 20.1)
        self.assertEqual(selected_rows[0]["selected_profile"], "candidate")
        self.assertEqual(selected_rows[1]["selected_profile"], "baseline")

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_dual_profile_selector_") as temp_dir:
            temp_root = Path(temp_dir)
            out_pos = temp_root / "selected.pos"
            ppc_dual_profile_selector.write_pos(out_pos, selected)
            reparsed = comparison.read_libgnss_pos(out_pos)
            metrics = ppc_metrics.summarize_solution_epochs(
                reference,
                reparsed,
                fixed_status=4,
                label="selected",
                match_tolerance_s=0.25,
                solver_wall_time_s=None,
            )
            self.assertEqual(metrics["ppc_official_score_pct"], 100.0)


class PPCDualProfileSelectorMatrixTest(unittest.TestCase):
    @staticmethod
    def summary(
        total_distance_m: float,
        baseline_score_m: float,
        candidate_score_m: float,
        selected_score_m: float,
        selected_segments: int,
    ) -> dict[str, object]:
        def metrics(score_m: float, positioning: float, fix: float) -> dict[str, float]:
            return {
                "positioning_rate_pct": positioning,
                "fix_rate_pct": fix,
                "ppc_official_score_pct": 100.0 * score_m / total_distance_m,
                "ppc_official_score_distance_m": score_m,
                "ppc_official_total_distance_m": total_distance_m,
                "p95_h_m": 1.0,
                "max_h_m": 3.0,
            }

        return {
            "rule": (
                "candidate_status_name == FIXED AND "
                "candidate_baseline_m <= 9053.95 AND "
                "candidate_baseline_m >= 940.785 AND "
                "candidate_num_satellites >= 8"
            ),
            "baseline": metrics(baseline_score_m, 80.0, 60.0),
            "candidate": metrics(candidate_score_m, 81.0, 61.0),
            "metrics": metrics(selected_score_m, 82.5, 63.0),
            "delta_vs_baseline": {
                "positioning_rate_pct": 2.5,
                "fix_rate_pct": 3.0,
                "ppc_official_score_pct": 100.0 * (selected_score_m - baseline_score_m) / total_distance_m,
                "ppc_official_score_distance_m": selected_score_m - baseline_score_m,
                "p95_h_m": -0.2,
            },
            "delta_vs_candidate": {
                "positioning_rate_pct": 1.5,
                "fix_rate_pct": 2.0,
                "ppc_official_score_pct": 100.0 * (selected_score_m - candidate_score_m) / total_distance_m,
                "ppc_official_score_distance_m": selected_score_m - candidate_score_m,
                "p95_h_m": -0.1,
            },
            "selection": {
                "baseline_selected_segments": 20,
                "candidate_selected_segments": selected_segments,
                "candidate_selected_gain_distance_m": max(0.0, selected_score_m - baseline_score_m),
                "candidate_selected_loss_distance_m": min(0.0, selected_score_m - baseline_score_m),
                "candidate_selected_score_delta_distance_m": selected_score_m - baseline_score_m,
                "segments": 30,
            },
        }

    def test_dual_selector_matrix_aggregates_weighted_scores(self) -> None:
        runs = [
            ppc_dual_selector_matrix.SelectorRun(
                key="tokyo_run1",
                label="Tokyo r1",
                rule="candidate_status_name == FIXED",
                baseline=self.summary(100.0, 50.0, 45.0, 60.0, 5)["baseline"],
                candidate=self.summary(100.0, 50.0, 45.0, 60.0, 5)["candidate"],
                selected=self.summary(100.0, 50.0, 45.0, 60.0, 5)["metrics"],
                delta_vs_baseline=self.summary(100.0, 50.0, 45.0, 60.0, 5)["delta_vs_baseline"],
                delta_vs_candidate=self.summary(100.0, 50.0, 45.0, 60.0, 5)["delta_vs_candidate"],
                selection=self.summary(100.0, 50.0, 45.0, 60.0, 5)["selection"],
            ),
            ppc_dual_selector_matrix.SelectorRun(
                key="nagoya_run1",
                label="Nagoya r1",
                rule="candidate_status_name == FIXED",
                baseline=self.summary(300.0, 150.0, 120.0, 180.0, 7)["baseline"],
                candidate=self.summary(300.0, 150.0, 120.0, 180.0, 7)["candidate"],
                selected=self.summary(300.0, 150.0, 120.0, 180.0, 7)["metrics"],
                delta_vs_baseline=self.summary(300.0, 150.0, 120.0, 180.0, 7)["delta_vs_baseline"],
                delta_vs_candidate=self.summary(300.0, 150.0, 120.0, 180.0, 7)["delta_vs_candidate"],
                selection=self.summary(300.0, 150.0, 120.0, 180.0, 7)["selection"],
            ),
        ]

        payload = ppc_dual_selector_matrix.build_payload(runs, "Selector matrix")
        aggregates = payload["aggregates"]

        self.assertEqual(aggregates["weighted_baseline_official_score_pct"], 50.0)
        self.assertEqual(aggregates["weighted_candidate_all_official_score_pct"], 41.25)
        self.assertEqual(aggregates["weighted_selector_official_score_pct"], 60.0)
        self.assertEqual(aggregates["selector_official_score_delta_m"], 40.0)
        self.assertEqual(aggregates["candidate_selected_segments"], 12)
        markdown = ppc_dual_selector_matrix.render_markdown(payload)
        self.assertIn("Selector matrix", markdown)
        self.assertIn("dual selector weighted official", markdown)

    def test_dual_selector_matrix_main_writes_json_markdown_and_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_dual_matrix_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_a = temp_root / "tokyo.json"
            summary_b = temp_root / "nagoya.json"
            output_json = temp_root / "matrix.json"
            output_md = temp_root / "matrix.md"
            output_png = temp_root / "matrix.png"
            summary_a.write_text(json.dumps(self.summary(100.0, 50.0, 45.0, 60.0, 5)), encoding="utf-8")
            summary_b.write_text(json.dumps(self.summary(300.0, 150.0, 120.0, 180.0, 7)), encoding="utf-8")

            argv = [
                "analyze_ppc_dual_profile_selector_matrix.py",
                "--run",
                f"tokyo_run1={summary_a}",
                "--run",
                f"nagoya_run1={summary_b}",
                "--summary-json",
                str(output_json),
                "--markdown-output",
                str(output_md),
                "--output-png",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_dual_selector_matrix.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_json.exists())
            self.assertTrue(output_md.exists())
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 780))
            except ModuleNotFoundError:
                pass


class PPCDualProfileSelectorDriverTest(unittest.TestCase):
    @staticmethod
    def write_reference_csv(path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["tow", "week", "lat", "lon", "height", "ecef_x", "ecef_y", "ecef_z"])
            for index in range(3):
                writer.writerow([float(index), 2300, 0.0, 0.0, 0.0, 10.0 * index, 0.0, 0.0])

    @staticmethod
    def solution_epoch(
        index: int,
        ecef_x_m: float,
        status: int,
        prefit_rms_m: float | None,
    ) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([ecef_x_m, 0.0, 0.0]),
            status,
            12,
            10.0 if status == 4 else 0.0,
            1000.0,
            2,
            16,
            8,
            8,
            0,
            prefit_rms_m,
            None if prefit_rms_m is None else prefit_rms_m * 4.0,
            prefit_rms_m,
            None if prefit_rms_m is None else prefit_rms_m * 4.0,
        )

    def test_driver_applies_selector_and_writes_matrix_outputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_dual_driver_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            baseline_dir = temp_root / "baseline"
            candidate_dir = temp_root / "candidate"
            output_dir = temp_root / "selected"
            matrix_json = temp_root / "matrix.json"
            matrix_md = temp_root / "matrix.md"
            matrix_png = temp_root / "matrix.png"

            baseline = [
                self.solution_epoch(0, 0.0, 4, None),
                self.solution_epoch(1, 11.2, 3, None),
                self.solution_epoch(2, 20.1, 4, None),
            ]
            candidate = [
                self.solution_epoch(0, 0.0, 4, 0.2),
                self.solution_epoch(1, 10.1, 4, 0.3),
                self.solution_epoch(2, 21.0, 3, 0.2),
            ]
            for city, run_name in (("tokyo", "run1"), ("nagoya", "run1")):
                key = f"{city}_{run_name}"
                self.write_reference_csv(dataset_root / city / run_name / "reference.csv")
                ppc_dual_profile_selector.write_pos(baseline_dir / f"{key}.pos", baseline)
                ppc_dual_profile_selector.write_pos(candidate_dir / f"{key}.pos", candidate)

            argv = [
                "run_ppc_dual_profile_selector_matrix.py",
                "--dataset-root",
                str(dataset_root),
                "--ppc-run",
                "tokyo/run1",
                "--ppc-run",
                "nagoya/run1",
                "--baseline-pos-template",
                str(baseline_dir / "{key}.pos"),
                "--candidate-pos-template",
                str(candidate_dir / "{key}.pos"),
                "--run-output-template",
                str(output_dir / "{key}_selected.pos"),
                "--rule",
                "candidate_status_name == FIXED AND "
                "candidate_rtk_update_prefit_residual_rms_m >= 0.2",
                "--matrix-summary-json",
                str(matrix_json),
                "--matrix-markdown-output",
                str(matrix_md),
                "--matrix-output-png",
                str(matrix_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_dual_selector_driver.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue((output_dir / "tokyo_run1_selected.pos").exists())
            self.assertTrue((output_dir / "tokyo_run1_selected_summary.json").exists())
            self.assertTrue((output_dir / "tokyo_run1_selected_segments.csv").exists())
            self.assertTrue(matrix_json.exists())
            self.assertTrue(matrix_md.exists())
            self.assertTrue(matrix_png.exists())
            payload = json.loads(matrix_json.read_text(encoding="utf-8"))
            self.assertGreater(payload["aggregates"]["selector_official_score_delta_m"], 0.0)
            self.assertIn("PPC dual-profile selector", matrix_md.read_text(encoding="utf-8"))


class PPCIMUCoverageTest(unittest.TestCase):
    def test_imu_coverage_summarizes_timing_overlap_and_loss_pool(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_imu_coverage_") as temp_dir:
            temp_root = Path(temp_dir)
            run_root = temp_root / "tokyo" / "run1"
            run_root.mkdir(parents=True)
            (run_root / "imu.csv").write_text(
                "\n".join(
                    [
                        "GPS TOW (s), GPS Week, Acc X (m/s^2), Acc Y (m/s^2), Acc Z (m/s^2), Ang Rate X (deg/s), Ang Rate Y (deg/s), Ang Rate Z (deg/s)",
                        "10.00,2324,0.0,0.0,9.8,0.1,0.2,0.3",
                        "10.01,2324,0.1,0.0,9.8,0.1,0.2,0.4",
                        "10.02,2324,0.0,0.1,9.8,0.1,0.2,0.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            (run_root / "reference.csv").write_text(
                "\n".join(
                    [
                        "GPS TOW (s),GPS Week,Latitude (deg),Longitude (deg),Ellipsoid Height (m)",
                        "10.00,2324,35.0,139.0,10.0",
                        "10.02,2324,35.0,139.0,10.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            quality_dir = temp_root / "quality"
            quality_dir.mkdir()
            (quality_dir / "tokyo_run1.json").write_text(
                json.dumps(
                    {
                        "official_loss_by_state": [
                            {"score_state": "scored", "distance_m": 5.0},
                            {"score_state": "high_error", "distance_m": 2.0},
                            {"score_state": "no_solution", "distance_m": 3.0},
                        ]
                    }
                ),
                encoding="ascii",
            )

            payload = ppc_imu_coverage.build_payload(
                temp_root,
                [ppc_imu_coverage.RunSpec("tokyo", "run1")],
                quality_json_template=str(quality_dir / "{key}.json"),
                target_score_pct=80.0,
            )

            aggregates = payload["aggregates"]
            self.assertEqual(aggregates["ready_run_count"], 1)
            self.assertEqual(aggregates["median_imu_rate_hz"], 100.0)
            self.assertEqual(aggregates["target_gap_distance_m"], 3.0)
            self.assertEqual(aggregates["no_solution_share_of_target_gap_pct"], 100.0)
            self.assertIn("Tokyo r1", ppc_imu_coverage.render_markdown(payload))


class PPCIMUBridgeTargetsTest(unittest.TestCase):
    def write_segments_csv(self, path: Path) -> None:
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(
                handle,
                fieldnames=[
                    "reference_index",
                    "start_tow_s",
                    "end_tow_s",
                    "segment_distance_m",
                    "lib_score_state",
                    "lib_status_name",
                    "lib_error_3d_m",
                ],
                lineterminator="\n",
            )
            writer.writeheader()
            rows = [
                (1, 0.0, 1.0, 10.0, "scored", "FIXED", 0.1),
                (2, 1.0, 1.5, 5.0, "no_solution", "", None),
                (3, 1.5, 2.0, 10.0, "scored", "FIXED", 0.1),
                (4, 2.0, 5.0, 8.0, "no_solution", "", None),
                (5, 5.0, 6.0, 10.0, "scored", "FLOAT", 0.2),
                (6, 6.0, 7.0, 7.0, "high_error", "FLOAT", 2.0),
            ]
            for reference_index, start, end, distance, state, status, error in rows:
                writer.writerow(
                    {
                        "reference_index": reference_index,
                        "start_tow_s": start,
                        "end_tow_s": end,
                        "segment_distance_m": distance,
                        "lib_score_state": state,
                        "lib_status_name": status,
                        "lib_error_3d_m": "" if error is None else error,
                    }
                )

    def test_bridge_targets_report_gap_limited_upper_bound(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_imu_bridge_targets_") as temp_dir:
            temp_root = Path(temp_dir)
            segment_csv = temp_root / "tokyo_run1_official_segments.csv"
            self.write_segments_csv(segment_csv)

            payload = ppc_imu_bridge_targets.build_payload(
                str(temp_root / "{key}_official_segments.csv"),
                [ppc_imu_bridge_targets.RunSpec("tokyo", "run1")],
                [1.0, 3.0],
            )

            aggregates = payload["aggregates"]
            self.assertEqual(aggregates["baseline_score_pct"], 60.0)
            self.assertEqual(aggregates["no_solution_span_count"], 2)
            rows = aggregates["bridge_thresholds"]
            self.assertEqual(rows[0]["recovered_no_solution_distance_m"], 5.0)
            self.assertEqual(rows[0]["score_pct"], 70.0)
            self.assertEqual(rows[1]["recovered_no_solution_distance_m"], 13.0)
            self.assertEqual(rows[1]["score_pct"], 86.0)
            self.assertEqual(aggregates["high_error_by_status"][0]["status_name"], "FLOAT")
            self.assertIn("PPC IMU Bridge Targets", ppc_imu_bridge_targets.render_markdown(payload))


class PPCCVDropoutBridgeMatrixTest(unittest.TestCase):
    @staticmethod
    def reference_epoch(index: int) -> comparison.ReferenceEpoch:
        return comparison.ReferenceEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([10.0 * index, 0.0, 0.0]),
        )

    @staticmethod
    def solution_epoch(index: int) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([10.0 * index, 0.0, 0.0]),
            4,
            12,
        )

    def write_reference_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="ascii", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "GPS TOW (s)",
                    "GPS Week",
                    "Latitude (deg)",
                    "Longitude (deg)",
                    "Ellipsoid Height (m)",
                    "ECEF X (m)",
                    "ECEF Y (m)",
                    "ECEF Z (m)",
                ]
            )
            for index in range(5):
                writer.writerow([float(index), 2300, 0.0, 0.0, 0.0, 10.0 * index, 0.0, 0.0])

    def test_cv_bridge_recovers_causal_linear_dropout(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_cv_bridge_matrix_") as temp_dir:
            temp_root = Path(temp_dir)
            self.write_reference_csv(temp_root / "tokyo" / "run1" / "reference.csv")
            baseline_dir = temp_root / "baseline"
            output_dir = temp_root / "out"
            baseline_epochs = [
                self.solution_epoch(0),
                self.solution_epoch(1),
                self.solution_epoch(4),
            ]
            ppc_dual_profile_selector.write_pos(baseline_dir / "tokyo_run1.pos", baseline_epochs)

            config = ppc_cv_bridge_matrix.BridgeConfig(
                max_gap_s=3.0,
                max_anchor_age_s=1.0,
                max_velocity_baseline_s=1.0,
                bridge_status=3,
                bridge_num_satellites=0,
                match_tolerance_s=0.25,
                threshold_m=0.50,
            )
            run_payload = ppc_cv_bridge_matrix.summarize_run(
                temp_root,
                ppc_cv_bridge_matrix.RunSpec("tokyo", "run1"),
                str(baseline_dir / "{key}.pos"),
                str(output_dir / "{key}_cv_bridge.pos"),
                None,
                config,
            )
            matrix = ppc_cv_bridge_matrix.build_matrix_payload(
                [run_payload],
                "PPC causal CV dropout bridge",
                config,
            )

            self.assertEqual(run_payload["baseline"]["ppc_official_score_pct"], 50.0)
            self.assertEqual(run_payload["metrics"]["ppc_official_score_pct"], 100.0)
            self.assertEqual(run_payload["selection"]["bridge_span_count"], 1)
            self.assertEqual(run_payload["selection"]["generated_epochs"], 2)
            self.assertEqual(run_payload["selection"]["recovered_distance_m"], 20.0)
            self.assertIn("PPC causal CV dropout bridge", ppc_cv_bridge_matrix.render_markdown(matrix))

    def test_cv_bridge_can_use_telemetry_anchor_mode(self) -> None:
        reference = [self.reference_epoch(0)]
        high_error_fixed = comparison.SolutionEpoch(
            2300,
            0.0,
            0.0,
            0.0,
            0.0,
            np.array([2.0, 0.0, 0.0]),
            4,
            12,
            12.0,
        )
        scored_config = ppc_cv_bridge_matrix.BridgeConfig(
            max_gap_s=1.0,
            max_anchor_age_s=1.0,
            max_velocity_baseline_s=1.0,
            bridge_status=3,
            bridge_num_satellites=0,
            match_tolerance_s=0.25,
            threshold_m=0.50,
        )
        telemetry_config = ppc_cv_bridge_matrix.BridgeConfig(
            max_gap_s=1.0,
            max_anchor_age_s=1.0,
            max_velocity_baseline_s=1.0,
            bridge_status=3,
            bridge_num_satellites=0,
            match_tolerance_s=0.25,
            threshold_m=0.50,
            anchor_mode="telemetry",
            anchor_statuses=(4,),
            anchor_min_ratio=10.0,
        )

        self.assertEqual(
            ppc_cv_bridge_matrix.trusted_anchor_epochs(
                reference,
                [high_error_fixed],
                scored_config,
            ),
            [],
        )
        self.assertEqual(
            ppc_cv_bridge_matrix.trusted_anchor_epochs(
                reference,
                [high_error_fixed],
                telemetry_config,
            ),
            [high_error_fixed],
        )

    def test_cv_bridge_innovation_anchor_rejects_predicted_jump(self) -> None:
        reference = [self.reference_epoch(index) for index in range(3)]
        anchors = [
            self.solution_epoch(0),
            self.solution_epoch(1),
            comparison.SolutionEpoch(
                2300,
                2.0,
                0.0,
                0.0,
                0.0,
                np.array([200.0, 0.0, 0.0]),
                4,
                12,
                12.0,
            ),
        ]
        config = ppc_cv_bridge_matrix.BridgeConfig(
            max_gap_s=1.0,
            max_anchor_age_s=1.0,
            max_velocity_baseline_s=1.0,
            bridge_status=3,
            bridge_num_satellites=0,
            match_tolerance_s=0.25,
            threshold_m=0.50,
            anchor_mode="innovation",
            anchor_statuses=(4,),
            anchor_min_ratio=10.0,
            anchor_max_innovation_m=5.0,
        )

        trusted = ppc_cv_bridge_matrix.trusted_anchor_epochs(reference, anchors, config)

        self.assertEqual([epoch.tow for epoch in trusted], [0.0, 1.0])

    def test_cv_bridge_innovation_anchor_accepts_predicted_fixed(self) -> None:
        reference = [self.reference_epoch(index) for index in range(3)]
        anchors = [
            self.solution_epoch(0),
            self.solution_epoch(1),
            comparison.SolutionEpoch(
                2300,
                2.0,
                0.0,
                0.0,
                0.0,
                np.array([20.2, 0.0, 0.0]),
                4,
                12,
                12.0,
            ),
        ]
        config = ppc_cv_bridge_matrix.BridgeConfig(
            max_gap_s=1.0,
            max_anchor_age_s=1.0,
            max_velocity_baseline_s=1.0,
            bridge_status=3,
            bridge_num_satellites=0,
            match_tolerance_s=0.25,
            threshold_m=0.10,
            anchor_mode="innovation",
            anchor_statuses=(4,),
            anchor_min_ratio=10.0,
            anchor_max_innovation_m=5.0,
        )

        trusted = ppc_cv_bridge_matrix.trusted_anchor_epochs(reference, anchors, config)

        self.assertEqual([epoch.tow for epoch in trusted], [0.0, 1.0, 2.0])


class PPCIMUDropoutBridgeMatrixTest(unittest.TestCase):
    @staticmethod
    def solution_epoch(index: int) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([6378137.0, 10.0 * index, 0.0]),
            4,
            12,
        )

    def write_reference_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="ascii", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "GPS TOW (s)",
                    "GPS Week",
                    "Latitude (deg)",
                    "Longitude (deg)",
                    "Ellipsoid Height (m)",
                    "ECEF X (m)",
                    "ECEF Y (m)",
                    "ECEF Z (m)",
                ]
            )
            for index in range(5):
                writer.writerow([float(index), 2300, 0.0, 0.0, 0.0, 6378137.0, 10.0 * index, 0.0])

    def write_imu_csv(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="ascii", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "GPS TOW (s)",
                    "GPS Week",
                    "Acc X (m/s^2)",
                    "Acc Y (m/s^2)",
                    "Acc Z (m/s^2)",
                ]
            )
            for index in range(5):
                writer.writerow([float(index), 2300, 0.0, 0.0, 9.8])

    def test_imu_bridge_recovers_linear_dropout_with_zero_accel(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_imu_bridge_matrix_") as temp_dir:
            temp_root = Path(temp_dir)
            self.write_reference_csv(temp_root / "tokyo" / "run1" / "reference.csv")
            self.write_imu_csv(temp_root / "tokyo" / "run1" / "imu.csv")
            baseline_dir = temp_root / "baseline"
            output_dir = temp_root / "out"
            baseline_epochs = [
                self.solution_epoch(0),
                self.solution_epoch(1),
                self.solution_epoch(4),
            ]
            ppc_dual_profile_selector.write_pos(baseline_dir / "tokyo_run1.pos", baseline_epochs)

            config = ppc_imu_bridge_matrix.IMUBridgeConfig(
                max_gap_s=3.0,
                max_anchor_age_s=1.0,
                max_velocity_baseline_s=1.0,
                bridge_status=3,
                bridge_num_satellites=0,
                match_tolerance_s=0.25,
                threshold_m=0.50,
                bias_window_s=1.0,
                min_heading_speed_mps=0.5,
                max_horizontal_accel_mps2=3.0,
                forward_axis="x",
                lateral_axis="y",
                forward_sign=1.0,
                lateral_sign=1.0,
            )
            run_payload = ppc_imu_bridge_matrix.summarize_run(
                temp_root,
                ppc_cv_bridge_matrix.RunSpec("tokyo", "run1"),
                str(baseline_dir / "{key}.pos"),
                str(output_dir / "{key}_imu_bridge.pos"),
                None,
                config,
            )
            matrix = ppc_imu_bridge_matrix.build_matrix_payload(
                [run_payload],
                "PPC causal IMU dropout bridge",
                config,
            )

            self.assertEqual(run_payload["baseline"]["ppc_official_score_pct"], 50.0)
            self.assertEqual(run_payload["metrics"]["ppc_official_score_pct"], 100.0)
            self.assertEqual(run_payload["selection"]["bridge_span_count"], 1)
            self.assertEqual(run_payload["selection"]["generated_epochs"], 2)
            self.assertEqual(run_payload["selection"]["recovered_distance_m"], 20.0)
            self.assertIn("PPC causal IMU dropout bridge", ppc_imu_bridge_matrix.render_markdown(matrix))


class PPCMetricsTest(unittest.TestCase):
    def test_libgnss_pos_parser_keeps_ratio_and_baseline_telemetry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_pos_ratio_parse_") as temp_dir:
            pos_path = Path(temp_dir) / "solution.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "% GPS_Week GPS_TOW X Y Z Lat Lon Height Status NumSat PDOP Ratio Baseline",
                        "2300 1.000 10.0 0.0 0.0 0.0 0.0 0.0 4 12 2.0 "
                        "17.5 9400.25 2 16 8 8 1 2.5 31.0 1.2 20.0 64.0 4.0 1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            epochs = comparison.read_libgnss_pos(pos_path)

            self.assertEqual(len(epochs), 1)
            self.assertEqual(epochs[0].ratio, 17.5)
            self.assertEqual(epochs[0].baseline_m, 9400.25)
            self.assertEqual(epochs[0].rtk_iterations, 2)
            self.assertEqual(epochs[0].rtk_update_observations, 16)
            self.assertEqual(epochs[0].rtk_update_phase_observations, 8)
            self.assertEqual(epochs[0].rtk_update_code_observations, 8)
            self.assertEqual(epochs[0].rtk_update_suppressed_outliers, 1)
            self.assertEqual(epochs[0].rtk_update_prefit_residual_rms_m, 2.5)
            self.assertEqual(epochs[0].rtk_update_prefit_residual_max_m, 31.0)
            self.assertEqual(epochs[0].rtk_update_post_suppression_residual_rms_m, 1.2)
            self.assertEqual(epochs[0].rtk_update_post_suppression_residual_max_m, 20.0)
            self.assertEqual(epochs[0].rtk_update_normalized_innovation_squared, 64.0)
            self.assertEqual(
                epochs[0].rtk_update_normalized_innovation_squared_per_observation,
                4.0,
            )
            self.assertEqual(epochs[0].rtk_update_rejected_by_innovation_gate, 1)

    def test_rtklib_pos_parser_keeps_ratio_telemetry(self) -> None:
        with tempfile.TemporaryDirectory(prefix="rtklib_pos_ratio_parse_") as temp_dir:
            pos_path = Path(temp_dir) / "rtklib.pos"
            pos_path.write_text(
                "\n".join(
                    [
                        "%  GPST latitude(deg) longitude(deg) height(m) Q ns sdn sde sdu sdne sdeu sdun age ratio",
                        "2024/07/20 10:22:00.000 35.165452362 136.881445510 "
                        "41.1780 1 7 0.0049 0.0044 0.0135 0.0026 0.0030 "
                        "0.0031 0.00 8.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            epochs = comparison.read_rtklib_pos(pos_path)

            self.assertEqual(len(epochs), 1)
            self.assertEqual(epochs[0].ratio, 8.5)
            self.assertIsNone(epochs[0].baseline_m)

    def test_official_distance_score_weights_reference_distance(self) -> None:
        reference = [
            comparison.ReferenceEpoch(2300, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 1.0, 0.0, 0.0, 0.0, (10.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 2.0, 0.0, 0.0, 0.0, (20.0, 0.0, 0.0)),
            comparison.ReferenceEpoch(2300, 3.0, 0.0, 0.0, 0.0, (40.0, 0.0, 0.0)),
        ]
        solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, (10.2, 0.0, 0.0), 4, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, (21.0, 0.0, 0.0), 4, 12),
        ]

        score = ppc_metrics.ppc_official_distance_score(reference, solution, 0.25)

        self.assertEqual(score["ppc_official_total_distance_m"], 40.0)
        self.assertEqual(score["ppc_official_matched_distance_m"], 20.0)
        self.assertEqual(score["ppc_official_score_distance_m"], 10.0)
        self.assertEqual(score["ppc_official_score_pct"], 25.0)

        records = ppc_metrics.ppc_official_segment_records(reference, solution, 0.25)
        self.assertEqual([record["score_state"] for record in records], ["scored", "high_error", "no_solution"])
        self.assertEqual(records[0]["status"], 4)
        self.assertEqual(records[1]["status"], 4)
        self.assertIsNone(records[2]["status"])


class PPCCoverageReadmeUpdateTest(unittest.TestCase):
    def sample_summary(self) -> dict[str, object]:
        return {
            "runs": [
                {
                    "key": "tokyo_run1",
                    "metrics": {
                        "positioning_rate_pct": 86.2,
                        "fix_rate_pct": 48.6,
                        "ppc_official_score_pct": 42.0,
                    },
                    "rtklib": {
                        "positioning_rate_pct": 66.3,
                        "fix_rate_pct": 30.5,
                        "ppc_official_score_pct": 21.0,
                    },
                    "delta_vs_rtklib": {
                        "positioning_rate_pct": 19.9,
                        "ppc_official_score_pct": 21.0,
                        "p95_h_m": -6.97,
                    },
                },
                {
                    "key": "nagoya_run1",
                    "metrics": {
                        "positioning_rate_pct": 87.9,
                        "fix_rate_pct": 60.3,
                        "ppc_official_score_pct": 50.0,
                    },
                    "rtklib": {
                        "positioning_rate_pct": 65.8,
                        "fix_rate_pct": 33.8,
                        "ppc_official_score_pct": 25.0,
                    },
                    "delta_vs_rtklib": {
                        "positioning_rate_pct": 22.1,
                        "ppc_official_score_pct": 25.0,
                        "p95_h_m": -22.63,
                    },
                },
            ]
        }

    def test_render_coverage_block_formats_table_and_averages(self) -> None:
        block = ppc_coverage_readme.render_coverage_block(self.sample_summary())

        self.assertIn("| Tokyo run1 | **86.2%** | 66.3% | **+19.9 pp** |", block)
        self.assertIn("| Nagoya run1 | **87.9%** | 65.8% | **+22.1 pp** |", block)
        self.assertIn("PPC official score", block)
        self.assertIn("Across these two public runs", block)
        self.assertIn("**+21.0 pp**", block)
        self.assertIn("**+23.0 pp** PPC official-score lead", block)
        self.assertIn("**-14.80 m** P95", block)

    def test_replace_marked_block_keeps_surrounding_markdown(self) -> None:
        original = "\n".join(
            [
                "before",
                ppc_coverage_readme.START_MARKER,
                "old generated block",
                ppc_coverage_readme.END_MARKER,
                "after",
            ]
        )

        updated = ppc_coverage_readme.replace_marked_block(original, "new generated block")

        self.assertIn("before", updated)
        self.assertIn("after", updated)
        self.assertIn("new generated block", updated)
        self.assertNotIn("old generated block", updated)

    def test_check_mode_reports_stale_target_without_writing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_readme_update_") as temp_dir:
            target = Path(temp_dir) / "README.md"
            original = "\n".join(
                [
                    ppc_coverage_readme.START_MARKER,
                    "old generated block",
                    ppc_coverage_readme.END_MARKER,
                    "",
                ]
            )
            target.write_text(original, encoding="utf-8")

            changed = ppc_coverage_readme.update_target(target, "new generated block", check=True)

            self.assertTrue(changed)
            self.assertEqual(target.read_text(encoding="utf-8"), original)

    def test_missing_rtklib_delta_fails(self) -> None:
        payload = self.sample_summary()
        runs = payload["runs"]
        assert isinstance(runs, list)
        assert isinstance(runs[0], dict)
        del runs[0]["delta_vs_rtklib"]

        with self.assertRaises(SystemExit):
            ppc_coverage_readme.render_coverage_block(payload)


class PPCCommercialHelpersTest(unittest.TestCase):
    def test_build_commercial_rover_command_keeps_tuning_local(self) -> None:
        solve = ppc_commercial.CommercialRoverSolve(
            rover=Path("rover_trimble.obs"),
            base=Path("base_trimble.obs"),
            nav=Path("base.nav"),
            out=Path("trimble.pos"),
            max_epochs=120,
            tuning=ppc_commercial.CommercialRoverTuning(
                preset="survey",
                arfilter=False,
                arfilter_margin=0.2,
                min_hold_count=4,
                hold_ratio_threshold=2.0,
            ),
        )

        command = ppc_commercial.build_commercial_rover_command(
            [sys.executable, str(ROOT_DIR / "apps" / "gnss.py")],
            solve,
        )

        self.assertEqual(command[:3], [sys.executable, str(ROOT_DIR / "apps" / "gnss.py"), "solve"])
        self.assertIn("--rover", command)
        self.assertIn("rover_trimble.obs", command)
        self.assertIn("--preset", command)
        self.assertIn("survey", command)
        self.assertIn("--no-arfilter", command)
        self.assertIn("--max-epochs", command)
        self.assertIn("120", command)

    def test_summarize_receiver_epochs_writes_isolated_match_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_commercial_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            reference = [
                comparison.ReferenceEpoch(
                    week=2300,
                    tow=1000.0,
                    lat_deg=35.0,
                    lon_deg=139.0,
                    height_m=42.0,
                    ecef=comparison.llh_to_ecef(35.0, 139.0, 42.0),
                )
            ]
            epochs = [
                comparison.SolutionEpoch(
                    week=2300,
                    tow=1000.0,
                    lat_deg=35.0000001,
                    lon_deg=139.0000001,
                    height_m=42.1,
                    ecef=comparison.llh_to_ecef(35.0000001, 139.0000001, 42.1),
                    status=4,
                    num_satellites=18,
                )
            ]
            match_csv = temp_root / "commercial_matches.csv"

            summary = ppc_commercial.summarize_receiver_epochs(
                reference=reference,
                epochs=epochs,
                label="trimble_net_r9",
                source="libgnss_solved_receiver_observations",
                solution_pos=temp_root / "trimble.pos",
                solution_format="pos",
                matched_csv=match_csv,
                match_tolerance_s=0.25,
                solver_wall_time_s=0.5,
                generated_solution=False,
                rover=temp_root / "rover_trimble.obs",
                base=temp_root / "base_trimble.obs",
                nav=temp_root / "base.nav",
            )

            self.assertEqual(summary["label"], "trimble_net_r9")
            self.assertEqual(summary["source"], "libgnss_solved_receiver_observations")
            self.assertFalse(summary["generated_solution"])
            self.assertEqual(summary["matched_epochs"], 1)
            self.assertTrue(match_csv.exists())
            self.assertIn("horizontal_error_m", match_csv.read_text(encoding="utf-8"))


class PublicRTKBenchmarksTest(unittest.TestCase):
    def test_matrix_keeps_urban_nav_as_tier_one_smoke(self) -> None:
        profiles = {
            profile.profile_id: profile for profile in public_rtk_benchmarks.PROFILES
        }

        urban_nav = profiles["urban-nav-tokyo"]
        smartloc = profiles["smartloc"]
        ppc = profiles["ppc-dataset"]

        self.assertEqual(ppc.status, "primary-public-rtk-signoff")
        self.assertIn("Septentrio mosaic-X5", ppc.receiver_artifacts)
        self.assertIn("proprietary receiver-engine solution", ppc.caveat)
        self.assertEqual(urban_nav.status, "wired-path-overrides")
        self.assertEqual(urban_nav.role, "tier-1 public smoke")
        self.assertIn("not the Trimble RTK engine", urban_nav.caveat)
        self.assertIn("--commercial-rover", urban_nav.adapter)
        self.assertEqual(smartloc.status, "receiver-fix-signoff")
        self.assertIn("smartloc-adapter", smartloc.adapter)

    def test_matrix_separates_candidate_adapters(self) -> None:
        candidates = public_rtk_benchmarks.select_profiles(["candidate"])
        candidate_ids = {profile.profile_id for profile in candidates}
        markdown = public_rtk_benchmarks.render_markdown(candidates)

        self.assertIn("gsdc", candidate_ids)
        self.assertIn("Ford Highway Driving RTK", markdown)
        self.assertNotIn("urban-nav-tokyo", candidate_ids)
        self.assertNotIn("smartloc", candidate_ids)


class SmartLocAdapterTest(unittest.TestCase):
    def test_convert_nav_posllh_exports_existing_comparison_contracts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_adapter_") as temp_dir:
            temp_root = Path(temp_dir)
            nav_posllh = temp_root / "NAV-POSLLH.csv"
            reference_csv = temp_root / "reference.csv"
            receiver_csv = temp_root / "receiver.csv"
            summary_json = temp_root / "summary.json"
            nav_posllh.write_text(
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon Cov) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat Cov) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];GPS time of week of the navigation epoch (iTOW) [ms];Longitude (lon) [deg];Latitude (lat) [deg];Height above ellipsoid (height) [m];Height above mean sea level (hMSL) [m];Horizontal accuracy estimate (hAcc) [m];Vertical accuracy estimate (vAcc) [m]",
                        "1900;126641.5;13.373657763;0.0;52.504560275;0.0;76.004611;0.0;1.24;0.0;0.9;0.0;5.7;0.0;0.002;0.0;126641500;13.3736776;52.5045750;80.242;38.043;0.547;-1",
                        "1900;126641.7;13.373662801;0.0;52.504570098;0.0;76.010967;0.0;1.23;0.0;0.8;0.0;5.8;0.0;-0.007;0.0;126641700;13.3736829;52.5045848;80.234;38.035;0.547;-1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            summary = smartloc_adapter.convert_nav_posllh(
                source_path=nav_posllh,
                reference_csv=reference_csv,
                receiver_csv=receiver_csv,
                receiver_label="smartloc_ublox",
                summary_json=summary_json,
            )

            self.assertEqual(summary["epochs"], 2)
            self.assertEqual(summary["adapter_status"], "receiver_csv_adapter")
            self.assertTrue(reference_csv.exists())
            self.assertTrue(receiver_csv.exists())
            self.assertTrue(summary_json.exists())
            reference = ppc_demo.read_flexible_reference_csv(reference_csv)
            receiver_records, receiver_format = moving_base_signoff.read_commercial_solution_records(
                receiver_csv,
                "csv",
            )
            self.assertEqual(len(reference), 2)
            self.assertEqual(len(receiver_records), 2)
            self.assertEqual(receiver_format, "csv")
            self.assertEqual(receiver_records[0]["status"], 1)

    def test_convert_rawx_exports_csv_and_rinex_observations(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_rawx_adapter_") as temp_dir:
            temp_root = Path(temp_dir)
            rawx = temp_root / "RXM-RAWX.csv"
            raw_csv = temp_root / "rawx.csv"
            obs_rinex = temp_root / "rover.obs"
            summary_json = temp_root / "raw_summary.json"
            rawx.write_text(
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];Measurement time of week (rcvTow) [s];GPS week number (week) [weeks];GPS leap seconds (leapS) [s];Number of measurements to follow (numMeas) [];Receiver tracking status (recStat) [];Pseudorange measurement (prMes) [m];Carrier phase measurement (cpMes) [cycles];Doppler measurement (doMes) [Hz];GNSS identifier (gnssId) [];Satellite identifier (svId) [];Frequency slot - only Glonass (freqId) [];Carrier phase locktime counter (locktime) [ms];Carrier-to-noise density ratio (cno) [dbHz];Estimated pseudorange measurement standard deviation (prStdev) [m];Estimated carrier phase measurement standard deviation (cpStdev) [cycles];Estimated Doppler measurement standard deviation (doStdev) [Hz];Tracking status (trkStat) [];NLOS (0 == no, 1 == yes, # == No Information)",
                        "1900;126641.5;13.0;0.0;52.0;0.0;76.0;0.0;1.0;0.0;0.0;0.0;5.0;0.0;0.0;0.0;126641.5;1900;17;2;1;19834597.871;104231506.047;222.1658;GPS;12;0;64500;50;0.32;0.004;0.128;15;0",
                        "1900;126641.5;13.0;0.0;52.0;0.0;76.0;0.0;1.0;0.0;0.0;0.0;5.0;0.0;0.0;0.0;126641.5;1900;17;2;1;19784715.199;105797774.172;2219.386;Glonass;20;9;20400;39;2.56;0.012;1.024;7;1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            summary = smartloc_adapter.convert_rawx(
                source_path=rawx,
                raw_csv=raw_csv,
                obs_rinex=obs_rinex,
                summary_json=summary_json,
            )

            self.assertEqual(summary["adapter_status"], "rawx_rinex_adapter")
            self.assertEqual(summary["raw_epochs"], 1)
            self.assertEqual(summary["raw_observations"], 2)
            self.assertEqual(summary["nlos_observations"], 1)
            self.assertTrue(raw_csv.exists())
            self.assertTrue(obs_rinex.exists())
            self.assertIn("pseudorange_m", raw_csv.read_text(encoding="utf-8"))
            rinex_text = obs_rinex.read_text(encoding="utf-8")
            self.assertIn("SYS / # / OBS TYPES", rinex_text)
            self.assertIn("G12", rinex_text)
            self.assertIn("R20", rinex_text)
            self.assertTrue(summary_json.exists())


class SmartLocSignoffTest(unittest.TestCase):
    def test_download_cache_filename_preserves_public_zip_name(self) -> None:
        self.assertEqual(
            smartloc_signoff.download_cache_filename(
                "https://www.tu-chemnitz.de/projekt/smartLoc/gnss_dataset/berlin/scenario1/berlin1_potsdamer_platz.zip"
            ),
            "berlin1_potsdamer_platz.zip",
        )

    def test_main_writes_receiver_fix_summary_with_raw_provenance(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            nav_posllh = temp_root / "NAV-POSLLH.csv"
            rawx = temp_root / "RXM-RAWX.csv"
            output_dir = temp_root / "out"
            nav_posllh.write_text(
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon Cov) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat Cov) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];GPS time of week of the navigation epoch (iTOW) [ms];Longitude (lon) [deg];Latitude (lat) [deg];Height above ellipsoid (height) [m];Height above mean sea level (hMSL) [m];Horizontal accuracy estimate (hAcc) [m];Vertical accuracy estimate (vAcc) [m]",
                        "1900;126641.5;13.373657763;0.0;52.504560275;0.0;76.004611;0.0;1.24;0.0;0.9;0.0;5.7;0.0;0.002;0.0;126641500;13.3736578;52.5045603;76.104;38.043;0.547;-1",
                        "1900;126641.7;13.373662801;0.0;52.504570098;0.0;76.010967;0.0;1.23;0.0;0.8;0.0;5.8;0.0;-0.007;0.0;126641700;13.3736629;52.5045702;76.111;38.035;0.547;-1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rawx.write_text(
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];Measurement time of week (rcvTow) [s];GPS week number (week) [weeks];GPS leap seconds (leapS) [s];Number of measurements to follow (numMeas) [];Receiver tracking status (recStat) [];Pseudorange measurement (prMes) [m];Carrier phase measurement (cpMes) [cycles];Doppler measurement (doMes) [Hz];GNSS identifier (gnssId) [];Satellite identifier (svId) [];Frequency slot - only Glonass (freqId) [];Carrier phase locktime counter (locktime) [ms];Carrier-to-noise density ratio (cno) [dbHz];Estimated pseudorange measurement standard deviation (prStdev) [m];Estimated carrier phase measurement standard deviation (cpStdev) [cycles];Estimated Doppler measurement standard deviation (doStdev) [Hz];Tracking status (trkStat) [];NLOS (0 == no, 1 == yes, # == No Information)",
                        "1900;126641.5;13.0;0.0;52.0;0.0;76.0;0.0;1.0;0.0;0.0;0.0;5.0;0.0;0.0;0.0;126641.5;1900;17;1;1;19834597.871;104231506.047;222.1658;GPS;12;0;64500;50;0.32;0.004;0.128;15;0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            args = argparse.Namespace(
                input=None,
                input_url=smartloc_signoff.DEFAULT_SMARTLOC_ZIP_URL,
                download_cache_dir=temp_root / "cache",
                force_download=False,
                nav_posllh=nav_posllh,
                rawx=rawx,
                output_dir=output_dir,
                reference_csv=None,
                receiver_csv=None,
                raw_csv=None,
                obs_rinex=None,
                matched_csv=None,
                summary_json=None,
                receiver_label="smartloc_ublox",
                match_tolerance_s=0.25,
                max_rows=-1,
                raw_max_epochs=-1,
                skip_raw_export=False,
                require_matched_epochs_min=2,
                require_mean_h_max=1.0,
                require_median_h_max=None,
                require_p95_h_max=1.0,
                require_max_h_max=1.0,
                require_p95_up_max=1.0,
                require_raw_epochs_min=1,
                require_raw_observations_min=1,
                require_solver_inputs_available=False,
            )

            with mock.patch.object(smartloc_signoff, "parse_args", return_value=args):
                exit_code = smartloc_signoff.main()

            self.assertEqual(exit_code, 0)
            summary_path = output_dir / "smartloc_signoff_summary.json"
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "smartloc-receiver-fix")
            self.assertEqual(payload["receiver_fix"]["matched_epochs"], 2)
            self.assertEqual(payload["raw_adapter"]["raw_epochs"], 1)
            self.assertEqual(payload["solver_preflight"]["status"], "blocked")
            self.assertIn(
                "missing broadcast navigation RINEX in smartLoc input",
                payload["solver_preflight"]["rtk_blockers"],
            )
            self.assertFalse(payload["solver_signoff_available"])


class OptionalRTKSignoffScriptTest(unittest.TestCase):
    def test_build_step_plan_marks_missing_inputs_as_skipped(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_rtk_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            steps = ci_rtk_signoffs.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual(
                [step.slug for step in steps],
                ["ppc_nagoya_run1_rtk", "ppc_tokyo_run1_rtk", "scorpion_moving_base"],
            )
            self.assertTrue(all(step.command is None for step in steps))
            self.assertEqual(steps[0].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[1].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[2].skip_reason, "SCORPION moving-base input is unavailable.")

    def test_build_step_plan_uses_dataset_rtklib_and_scorpion_url(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_rtk_plan_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            dataset_root.mkdir()
            rtklib_bin = temp_root / "rnx2rtkp"
            rtklib_bin.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
            rtklib_bin.chmod(0o755)
            output_dir = temp_root / "output"
            env = {
                "GNSSPP_PPC_DATASET_ROOT": str(dataset_root),
                "GNSSPP_RTKLIB_BIN": str(rtklib_bin),
                "GNSSPP_SCORPION_MOVING_BASE_URL": "https://example.com/scorpion.ubx",
            }

            steps = ci_rtk_signoffs.build_step_plan(ROOT_DIR, output_dir, env)

            nagoya, tokyo, scorpion = steps
            self.assertIsNotNone(nagoya.command)
            self.assertIn("--dataset-root", nagoya.command)
            self.assertIn(str(dataset_root), nagoya.command)
            self.assertIn("--debug-epoch-log", nagoya.command)
            self.assertIsNotNone(tokyo.command)
            self.assertIn("--rtklib-bin", tokyo.command)
            self.assertIn(str(rtklib_bin), tokyo.command)
            self.assertIn("--rtklib-pos", tokyo.command)
            self.assertIsNotNone(scorpion.command)
            self.assertIn("--input-url", scorpion.command)
            self.assertIn("https://example.com/scorpion.ubx", scorpion.command)

    def test_build_step_plan_skips_tokyo_when_rtklib_is_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_rtk_missing_rtklib_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            dataset_root.mkdir()
            steps = ci_rtk_signoffs.build_step_plan(
                ROOT_DIR,
                temp_root / "output",
                {"GNSSPP_PPC_DATASET_ROOT": str(dataset_root)},
            )

            nagoya, tokyo, _ = steps
            self.assertIsNotNone(nagoya.command)
            self.assertIsNone(tokyo.command)
            self.assertEqual(tokyo.skip_reason, "RTKLIB binary is unavailable.")

    def test_run_step_writes_log_for_skipped_and_passed_steps(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_rtk_exec_") as temp_dir:
            temp_root = Path(temp_dir)
            log_dir = temp_root / "logs"
            log_dir.mkdir()

            skipped = ci_rtk_signoffs.SignoffStep(
                name="Skipped example",
                slug="skip_example",
                command=None,
                outputs=[],
                skip_reason="not configured",
            )
            skipped_result = ci_rtk_signoffs.run_step(skipped, ROOT_DIR, log_dir)
            self.assertEqual(skipped_result["status"], "skipped")
            self.assertTrue((log_dir / "skip_example.log").exists())

            passed = ci_rtk_signoffs.SignoffStep(
                name="Passed example",
                slug="pass_example",
                command=[sys.executable, "-c", "print('ok')"],
                outputs=[],
            )
            passed_result = ci_rtk_signoffs.run_step(passed, ROOT_DIR, log_dir)
            self.assertEqual(passed_result["status"], "passed")
            self.assertEqual(passed_result["returncode"], 0)
            self.assertIn("ok", (log_dir / "pass_example.log").read_text(encoding="utf-8"))

    def test_render_markdown_summary_reports_status_table(self) -> None:
        markdown = ci_rtk_signoffs.render_markdown_summary(
            [
                {
                    "name": "PPC Nagoya RTK sign-off",
                    "status": "passed",
                    "elapsed_s": 1.25,
                    "log_path": "/tmp/a.log",
                },
                {
                    "name": "PPC Tokyo RTK sign-off with RTKLIB comparison",
                    "status": "skipped",
                    "skip_reason": "RTKLIB binary is unavailable.",
                    "log_path": "/tmp/b.log",
                },
                {
                    "name": "SCORPION moving-base sign-off",
                    "status": "failed",
                    "log_path": "/tmp/c.log",
                },
            ]
        )

        self.assertIn("## Optional RTK Sign-offs", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`failed`: `1`", markdown)
        self.assertIn("`skipped`: `1`", markdown)
        self.assertIn("| PPC Nagoya RTK sign-off | `passed` | 1.25s |", markdown)
        self.assertIn("RTKLIB binary is unavailable.", markdown)
        self.assertIn("see `/tmp/c.log`", markdown)

class OptionalPPPProductsSignoffScriptTest(unittest.TestCase):
    def test_build_step_plan_skips_when_malib_is_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_ppp_skip_") as temp_dir:
            output_dir = Path(temp_dir) / "output"
            steps = ci_ppp_products_signoff.build_step_plan(ROOT_DIR, output_dir, {})

            self.assertEqual([step.slug for step in steps], ["ppp_kinematic_products"])
            self.assertIsNone(steps[0].command)
            self.assertEqual(steps[0].skip_reason, "MALIB binary is unavailable.")

    def test_build_step_plan_skips_when_inputs_are_missing(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_ppp_missing_inputs_") as temp_dir:
            temp_root = Path(temp_dir)
            malib_bin = temp_root / "malib"
            malib_bin.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
            malib_bin.chmod(0o755)

            steps = ci_ppp_products_signoff.build_step_plan(
                ROOT_DIR,
                temp_root / "output",
                {"GNSSPP_MALIB_BIN": str(malib_bin)},
            )

            self.assertIsNone(steps[0].command)
            self.assertIsNotNone(steps[0].skip_reason)
            self.assertIn("PPP products input is unavailable", str(steps[0].skip_reason))

    def test_build_step_plan_uses_configured_inputs_and_malib_config(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_ppp_plan_") as temp_dir:
            temp_root = Path(temp_dir)
            malib_bin = temp_root / "malib"
            malib_bin.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
            malib_bin.chmod(0o755)
            malib_config = temp_root / "malib.conf"
            obs = temp_root / "rover.obs"
            base = temp_root / "base.obs"
            nav = temp_root / "base.nav"
            for path in (malib_config, obs, base, nav):
                path.write_text("synthetic\n", encoding="ascii")
            output_dir = temp_root / "output"
            env = {
                "GNSSPP_MALIB_BIN": str(malib_bin),
                "GNSSPP_PPP_PRODUCTS_MALIB_CONFIG": str(malib_config),
                "GNSSPP_PPP_PRODUCTS_OBS": str(obs),
                "GNSSPP_PPP_PRODUCTS_BASE": str(base),
                "GNSSPP_PPP_PRODUCTS_NAV": str(nav),
            }

            steps = ci_ppp_products_signoff.build_step_plan(ROOT_DIR, output_dir, env)

            command = steps[0].command
            self.assertIsNotNone(command)
            assert command is not None
            self.assertIn("ppp-products-signoff", command)
            self.assertIn("--obs", command)
            self.assertIn(str(obs), command)
            self.assertIn("--base", command)
            self.assertIn(str(base), command)
            self.assertIn("--nav", command)
            self.assertIn(str(nav), command)
            self.assertIn("--malib-bin", command)
            self.assertIn(str(malib_bin), command)
            self.assertIn("--malib-config", command)
            self.assertIn(str(malib_config), command)
            self.assertIn(str(output_dir / "ppp_kinematic_products_summary.json"), command)

    def test_run_step_collects_metrics_from_summary_json(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_ppp_exec_") as temp_dir:
            temp_root = Path(temp_dir)
            log_dir = temp_root / "logs"
            log_dir.mkdir()
            summary_json = temp_root / "ppp_summary.json"
            payload = {
                "products_signoff_profile": "kinematic",
                "ppp_solution_rate_pct": 100.0,
                "common_epoch_pairs": 24,
                "p95_position_error_m": 0.42,
                "comparison_status": "better",
            }
            code = (
                "import json\n"
                "from pathlib import Path\n"
                f"Path({str(summary_json)!r}).write_text(json.dumps({payload!r}), encoding='utf-8')\n"
            )
            step = ci_ppp_products_signoff.SignoffStep(
                name="PPP products sign-off with MALIB comparison",
                slug="ppp_pass",
                command=[sys.executable, "-c", code],
                outputs=[],
                summary_json=str(summary_json),
            )

            result = ci_ppp_products_signoff.run_step(step, ROOT_DIR, log_dir)

            self.assertEqual(result["status"], "passed")
            self.assertEqual(result["returncode"], 0)
            self.assertEqual(result["metrics"], payload)

    def test_render_markdown_summary_reports_status_table_and_metrics(self) -> None:
        markdown = ci_ppp_products_signoff.render_markdown_summary(
            [
                {
                    "name": "PPP products sign-off with MALIB comparison",
                    "status": "passed",
                    "elapsed_s": 1.25,
                    "metrics": {
                        "products_signoff_profile": "kinematic",
                        "ppp_solution_rate_pct": 100.0,
                        "common_epoch_pairs": 24,
                        "p95_position_error_m": 0.42,
                        "comparison_status": "better",
                    },
                },
                {
                    "name": "PPP products sign-off with MALIB comparison",
                    "status": "skipped",
                    "skip_reason": "MALIB binary is unavailable.",
                },
                {
                    "name": "PPP products sign-off with MALIB comparison",
                    "status": "failed",
                    "log_path": "/tmp/ppp.log",
                },
            ]
        )

        self.assertIn("## Optional PPP Products Sign-off", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`failed`: `1`", markdown)
        self.assertIn("`skipped`: `1`", markdown)
        self.assertIn("profile `kinematic`", markdown)
        self.assertIn("PPP solution 100%", markdown)
        self.assertIn("common pairs `24`", markdown)
        self.assertIn("p95 0.42 m", markdown)
        self.assertIn("comparison `better`", markdown)
        self.assertIn("MALIB binary is unavailable.", markdown)
        self.assertIn("see `/tmp/ppp.log`", markdown)


class CIScopeDetectionTest(unittest.TestCase):
    def test_classify_changed_paths_marks_docs_only_changes(self) -> None:
        payload = ci_scope.classify_changed_paths(
            [
                "docs/guide.md",
                "notes/2026-04-11_ci.md",
                "README.md",
                "scripts/generate_architecture_diagram.py",
            ]
        )

        self.assertEqual(
            payload["changed_paths"],
            [
                "README.md",
                "docs/guide.md",
                "notes/2026-04-11_ci.md",
                "scripts/generate_architecture_diagram.py",
            ],
        )
        self.assertTrue(payload["docs_only"])
        self.assertFalse(payload["run_heavy"])

    def test_classify_changed_paths_runs_heavy_when_code_changes_exist(self) -> None:
        payload = ci_scope.classify_changed_paths(
            [
                "docs/guide.md",
                "src/algorithms/rtk.cpp",
                ".github/workflows/ci.yml",
            ]
        )

        self.assertFalse(payload["docs_only"])
        self.assertTrue(payload["run_heavy"])

    def test_classify_changed_paths_runs_heavy_for_empty_diff(self) -> None:
        payload = ci_scope.classify_changed_paths([])

        self.assertEqual(payload["changed_paths"], [])
        self.assertFalse(payload["docs_only"])
        self.assertTrue(payload["run_heavy"])

    def test_render_markdown_summary_includes_paths_and_flags(self) -> None:
        markdown = ci_scope.render_markdown_summary(
            {
                "changed_paths": ["README.md", "docs/guide.md"],
                "docs_only": True,
                "run_heavy": False,
            }
        )

        self.assertIn("## CI Scope", markdown)
        self.assertIn("`docs_only`: `true`", markdown)
        self.assertIn("`run_heavy`: `false`", markdown)
        self.assertIn("`README.md`", markdown)
        self.assertIn("`docs/guide.md`", markdown)


class MovingBaseSignoffHelpersTest(unittest.TestCase):
    def test_read_reference_rows_accepts_ecef_columns(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_ref_") as temp_dir:
            reference_csv = Path(temp_dir) / "reference.csv"
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            rows = moving_base_signoff.read_reference_rows(reference_csv)

            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["week"], 2200.0)
            self.assertEqual(rows[0]["tow"], 345600.0)
            self.assertAlmostEqual(rows[0]["base_x"], 3875000.0)
            self.assertAlmostEqual(rows[0]["rover_y"], 332002.0)

    def test_match_solution_to_reference_reports_baseline_and_heading_errors(self) -> None:
        solution_records = [
            {
                "week": 2200,
                "tow": 345600.0,
                "x": 3875001.2,
                "y": 332002.1,
                "z": 5029000.4,
                "status": 4,
                "satellites": 12,
            }
        ]
        reference_rows = [
            {
                "week": 2200.0,
                "tow": 345600.0,
                "base_x": 3875000.0,
                "base_y": 332000.0,
                "base_z": 5029000.0,
                "rover_x": 3875001.0,
                "rover_y": 332002.0,
                "rover_z": 5029000.5,
            }
        ]

        matches = moving_base_signoff.match_solution_to_reference(
            solution_records, reference_rows, 0.25
        )

        self.assertEqual(len(matches), 1)
        self.assertGreater(matches[0]["baseline_error_m"], 0.0)
        self.assertGreater(matches[0]["baseline_length_m"], 0.0)
        self.assertIsNotNone(matches[0]["heading_error_deg"])

    def test_read_commercial_csv_records_accepts_receiver_solution_columns(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_commercial_rtk_") as temp_dir:
            commercial_csv = Path(temp_dir) / "receiver.csv"
            commercial_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m,fix_type,num_satellites",
                        "2200,345600.0,3875001.0,332002.0,5029000.5,rtk_fixed,14",
                        "2200,345601.0,3875001.1,332002.1,5029000.5,rtk_float,12",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            records, resolved_format = moving_base_signoff.read_commercial_solution_records(
                commercial_csv,
                "auto",
            )

            self.assertEqual(resolved_format, "csv")
            self.assertEqual(len(records), 2)
            self.assertEqual(records[0]["status"], 4)
            self.assertEqual(records[1]["status"], 3)
            self.assertEqual(records[0]["satellites"], 14)


class DrivingComparisonHelpersTest(unittest.TestCase):
    @staticmethod
    def matched_epoch(tow: float, horiz_error_m: float, up_m: float, status: int) -> comparison.MatchedEpoch:
        return comparison.MatchedEpoch(
            tow=tow,
            traj_east_m=tow,
            traj_north_m=0.0,
            traj_up_m=0.0,
            east_m=horiz_error_m,
            north_m=0.0,
            up_m=up_m,
            horiz_error_m=horiz_error_m,
            status=status,
        )

    def test_pair_epochs_builds_unique_common_epoch_pairs(self) -> None:
        lib = [
            self.matched_epoch(0.0, 0.7, 0.4, 4),
            self.matched_epoch(1.0, 0.8, 0.3, 3),
            self.matched_epoch(2.0, 0.9, 0.2, 3),
            self.matched_epoch(4.0, 1.0, 0.1, 1),
        ]
        rtklib = [
            self.matched_epoch(1.0, 0.6, 0.5, 1),
            self.matched_epoch(2.0, 0.7, 0.6, 2),
            self.matched_epoch(3.0, 0.8, 0.7, 5),
        ]

        pairs = comparison.pair_epochs(lib, rtklib, tolerance_s=0.11)

        self.assertEqual([round(pair.tow, 1) for pair in pairs], [1.0, 2.0])

        lib_common, rt_common = comparison.summarize_common_epochs(
            pairs,
            lib_fixed_status=4,
            rtklib_fixed_status=1,
        )
        self.assertEqual(lib_common["epochs"], 2)
        self.assertEqual(rt_common["epochs"], 2)
        self.assertAlmostEqual(lib_common["median_h_m"], 0.85)
        self.assertAlmostEqual(rt_common["median_h_m"], 0.65)

    def test_ppc_coverage_quality_groups_status_and_bad_segments(self) -> None:
        matches = [
            self.matched_epoch(0.0, 0.1, 0.1, 4),
            self.matched_epoch(1.0, 10.0, 1.0, 3),
            self.matched_epoch(2.0, 20.0, 2.0, 3),
            self.matched_epoch(2.5, 30.0, 3.0, 1),
            self.matched_epoch(3.0, 0.1, 0.1, 4),
        ]

        status_rows = ppc_coverage_quality.summarize_by_status(matches, reference_count=5)
        status_by_name = {row["status"]: row for row in status_rows}
        self.assertEqual(status_by_name["FIXED"]["epochs"], 2)
        self.assertEqual(status_by_name["FLOAT"]["epochs"], 2)
        self.assertEqual(status_by_name["SPP"]["epochs"], 1)
        self.assertEqual(status_by_name["FIXED"]["ppc_score_3d_50cm_epochs"], 2)

        global_p95, contribution = ppc_coverage_quality.p95_contribution_by_status(matches)
        self.assertGreater(global_p95, 20.0)
        self.assertEqual(contribution[0]["status"], "SPP")
        self.assertEqual(contribution[0]["epochs"], 1)

        segments = ppc_coverage_quality.bad_segments(
            matches,
            threshold_m=15.0,
            max_gap_s=0.6,
        )
        self.assertEqual(len(segments), 1)
        self.assertEqual(segments[0]["epochs"], 2)
        self.assertEqual(segments[0]["statuses"], ["FLOAT", "SPP"])
        self.assertEqual(segments[0]["status_counts"], {"FLOAT": 1, "SPP": 1})
        self.assertEqual(segments[0]["dominant_status"], "FLOAT")
        self.assertEqual(segments[0]["previous_fixed_tow_s"], 0.0)
        self.assertEqual(segments[0]["next_fixed_tow_s"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_gap_s"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_distance_m"], 3.0)
        self.assertEqual(segments[0]["fixed_anchor_speed_mps"], 1.0)
        self.assertEqual(segments[0]["fixed_anchor_bridge_residual_max_m"], 0.0)

        reference = [
            comparison.ReferenceEpoch(2300, 0.0, 0.0, 0.0, 0.0, np.array([0.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([10.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.0, 0.0, 0.0])),
            comparison.ReferenceEpoch(2300, 3.0, 0.0, 0.0, 0.0, np.array([40.0, 0.0, 0.0])),
        ]
        lib_solution = [
            comparison.SolutionEpoch(
                2300,
                1.0,
                0.0,
                0.0,
                0.0,
                np.array([10.2, 0.0, 0.0]),
                3,
                12,
                4.0,
                100.0,
                2,
                14,
                7,
                7,
                0,
                0.25,
                4.0,
                0.25,
                4.0,
                56.0,
                4.0,
                0,
            ),
            comparison.SolutionEpoch(
                2300,
                2.0,
                0.0,
                0.0,
                0.0,
                np.array([21.0, 0.0, 0.0]),
                3,
                12,
                12.0,
                101.0,
                2,
                16,
                8,
                8,
                1,
                2.5,
                31.0,
                1.2,
                20.0,
                128.0,
                8.0,
                1,
            ),
        ]
        rtklib_solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([11.0, 0.0, 0.0]), 2, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.1, 0.0, 0.0]), 1, 12),
        ]
        spp_solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([10.3, 0.0, 0.0]), 1, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.1, 0.0, 0.0]), 1, 12),
        ]
        lib_records = ppc_metrics.ppc_official_segment_records(reference, lib_solution, 0.25)
        rtklib_records = ppc_metrics.ppc_official_segment_records(reference, rtklib_solution, 0.25)
        spp_records = ppc_metrics.ppc_official_segment_records(reference, spp_solution, 0.25)

        loss_by_state = {
            row["score_state"]: row
            for row in ppc_coverage_quality.official_loss_by_state(lib_records)
        }
        self.assertEqual(loss_by_state["scored"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["high_error"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["no_solution"]["distance_m"], 20.0)

        high_error_by_status = {
            row["status"]: row
            for row in ppc_coverage_quality.official_loss_by_status(
                lib_records,
                ppc_coverage_quality.status_name,
                ("high_error",),
            )
        }
        self.assertEqual(high_error_by_status["FLOAT"]["distance_m"], 10.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_ratio"], 12.0)
        self.assertEqual(high_error_by_status["FLOAT"]["ratio_ge_10_distance_m"], 10.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_update_observations"], 16.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_prefit_rms_m"], 2.5)
        self.assertEqual(high_error_by_status["FLOAT"]["p95_rtk_prefit_max_m"], 31.0)
        self.assertEqual(high_error_by_status["FLOAT"]["median_rtk_update_nis_per_obs"], 8.0)
        self.assertEqual(high_error_by_status["FLOAT"]["rtk_update_nis_rejected_segments"], 1)
        self.assertEqual(high_error_by_status["FLOAT"]["rtk_update_nis_rejected_distance_m"], 10.0)
        rtk_diagnostics_by_state = {
            (row["status"], row["score_state"]): row
            for row in ppc_coverage_quality.official_rtk_update_diagnostics_by_state(
                lib_records,
                ppc_coverage_quality.status_name,
            )
        }
        self.assertEqual(rtk_diagnostics_by_state[("FLOAT", "scored")]["distance_m"], 10.0)
        self.assertEqual(rtk_diagnostics_by_state[("FLOAT", "high_error")]["distance_m"], 10.0)
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "scored")]["median_rtk_prefit_rms_m"],
            0.25,
        )
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "high_error")]["median_rtk_prefit_rms_m"],
            2.5,
        )
        self.assertEqual(
            rtk_diagnostics_by_state[("FLOAT", "high_error")][
                "median_rtk_update_nis_per_obs"
            ],
            8.0,
        )
        unscored_by_status = {
            row["status"]: row
            for row in ppc_coverage_quality.official_loss_by_status(
                lib_records,
                ppc_coverage_quality.status_name,
            )
        }
        self.assertEqual(unscored_by_status["NO_SOLUTION"]["distance_m"], 20.0)

        official_segments = ppc_coverage_quality.official_loss_segments(
            lib_records,
            ppc_coverage_quality.status_name,
        )
        self.assertEqual(len(official_segments), 1)
        self.assertEqual(official_segments[0]["distance_m"], 30.0)
        self.assertEqual(official_segments[0]["dominant_score_state"], "no_solution")
        self.assertEqual(official_segments[0]["status_counts"], {"NO_SOLUTION": 1, "FLOAT": 1})

        combined = ppc_coverage_quality.official_combined_records(lib_records, rtklib_records)
        delta_by_bucket = {
            row["bucket"]: row
            for row in ppc_coverage_quality.official_delta_by_bucket(combined)
        }
        self.assertEqual(delta_by_bucket["gnssplusplus_gain"]["score_delta_pct"], 25.0)
        self.assertEqual(delta_by_bucket["rtklib_gain"]["score_delta_pct"], -25.0)
        best_of_score = ppc_coverage_quality.official_best_of_lib_rtklib_score(
            ppc_metrics.ppc_official_distance_score(reference, lib_solution, 0.25),
            combined,
        )
        self.assertEqual(best_of_score["score_distance_m"], 20.0)
        self.assertEqual(best_of_score["score_pct"], 50.0)
        self.assertEqual(best_of_score["rtklib_additional_distance_m"], 10.0)
        self.assertEqual(best_of_score["remaining_unscored_distance_m"], 20.0)
        float_spp_sweep = ppc_coverage_quality.official_float_spp_divergence_sweep(
            lib_records,
            spp_records,
            lib_solution,
            spp_solution,
            [0.5],
        )
        self.assertEqual(float_spp_sweep[0]["threshold_m"], 0.5)
        self.assertEqual(float_spp_sweep[0]["score_distance_m"], 20.0)
        self.assertEqual(float_spp_sweep[0]["score_delta_distance_m"], 10.0)
        self.assertEqual(float_spp_sweep[0]["recovered_distance_m"], 10.0)
        self.assertEqual(combined[1]["lib_ratio"], 12.0)
        self.assertEqual(combined[1]["lib_baseline_m"], 101.0)
        self.assertEqual(combined[1]["lib_rtk_update_observations"], 16)
        self.assertEqual(combined[1]["lib_rtk_update_prefit_residual_max_m"], 31.0)
        self.assertEqual(
            combined[1]["lib_rtk_update_normalized_innovation_squared_per_observation"],
            8.0,
        )
        self.assertEqual(combined[1]["lib_rtk_update_rejected_by_innovation_gate"], 1)


class ScorecardRenderTest(unittest.TestCase):
    def write_reference_csv(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float]],
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "gps_tow_s",
                    "gps_week",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "ecef_x_m",
                    "ecef_y_m",
                    "ecef_z_m",
                ]
            )
            for week, tow, lat, lon, height in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                writer.writerow(
                    [tow, week, lat, lon, height, ecef[0], ecef[1], ecef[2]]
                )

    def write_lib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic libgnss++ solution\n")
            for week, tow, lat, lon, height, status in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                handle.write(
                    f"{week} {tow:.1f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} {status} 10 1.0\n"
                )

    def write_rtklib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic rtklib solution\n")
            for week, tow, lat, lon, height, status in rows:
                handle.write(
                    f"{week} {tow:.1f} {lat:.9f} {lon:.9f} {height:.4f} {status} 10\n"
                )

    def test_scorecard_main_renders_png_with_zero_fix_baseline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "scorecard.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.3, 4),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.5, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 5),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 5),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 5),
                ],
            )

            argv = [
                "generate_odaiba_scorecard.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic Odaiba",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    scorecard.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)

    def test_social_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_social_card_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "social_card.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.3, 4),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.5, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.2, 1),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 1),
                ],
            )

            argv = [
                "generate_odaiba_social_card.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic Odaiba",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    social_card.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1200, 630))
            except ModuleNotFoundError:
                pass

    def test_feature_overview_card_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_feature_card_test_") as temp_dir:
            output_png = Path(temp_dir) / "feature_overview.png"

            argv = [
                "generate_feature_overview_card.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    feature_overview.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (2240, 1376))
            except ModuleNotFoundError:
                pass

    def test_ppc_rtk_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_scorecard_test_") as temp_dir:
            output_png = Path(temp_dir) / "ppc_rtk_scorecard.png"

            argv = [
                "generate_ppc_rtk_scorecard.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 750))
            except ModuleNotFoundError:
                pass

    def test_ppc_tail_cleanup_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_tail_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            baseline_summary = temp_root / "baseline.json"
            cleanup_summary = temp_root / "cleanup.json"
            output_png = temp_root / "ppc_tail_cleanup_scorecard.png"

            def run_record(key: str, pos: float, official: float, p95: float, max_h: float, rejected: int) -> dict[str, object]:
                return {
                    "key": key,
                    "metrics": {
                        "positioning_rate_pct": pos,
                        "fix_rate_pct": 50.0,
                        "ppc_official_score_pct": official,
                        "p95_h_m": p95,
                        "max_h_m": max_h,
                    },
                    "delta_vs_rtklib": {
                        "p95_h_m": p95 - 30.0,
                    },
                    "guards": {
                        "nonfix_drift_guard": {"rejected_epochs": rejected},
                        "fixed_bridge_burst_guard": {"rejected_epochs": 2},
                    },
                }

            baseline_summary.write_text(
                json.dumps(
                    {
                        "runs": [
                            run_record("tokyo_run1", 90.0, 35.0, 34.0, 52.0, 0),
                            run_record("nagoya_run1", 88.0, 49.0, 12.0, 18.0, 0),
                        ]
                    }
                ),
                encoding="utf-8",
            )
            cleanup_summary.write_text(
                json.dumps(
                    {
                        "ratio": 2.4,
                        "nonfix_drift_max_residual": 4.0,
                        "fixed_bridge_burst_max_residual": 20.0,
                        "runs": [
                            run_record("tokyo_run1", 87.6, 34.9, 26.6, 47.3, 337),
                            run_record("nagoya_run1", 87.7, 48.9, 11.0, 16.5, 5),
                        ],
                    }
                ),
                encoding="utf-8",
            )

            argv = [
                "generate_ppc_tail_cleanup_scorecard.py",
                "--baseline-summary-json",
                str(baseline_summary),
                "--cleanup-summary-json",
                str(cleanup_summary),
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_tail_cleanup_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 760))
            except ModuleNotFoundError:
                pass

    def test_ppc_selector_validation_scorecard_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_selector_scorecard_test_") as temp_dir:
            temp_root = Path(temp_dir)
            net_summary = temp_root / "net.json"
            robust_summary = temp_root / "robust.json"
            output_png = temp_root / "ppc_selector_validation_scorecard.png"

            def summary(net_m: float, precision_pct: float, nonnegative: int) -> dict[str, object]:
                return {
                    "aggregates": {
                        "fold_count": 2,
                        "holdout_selected_score_delta_distance_m": net_m,
                        "holdout_selector_vs_candidate_all_delta_m": net_m + 20.0,
                        "holdout_distance_precision_pct": precision_pct,
                        "nonnegative_holdout_runs": nonnegative,
                        "min_holdout_delta_m": min(1.0, net_m),
                        "holdout_selected_loss_distance_m": -2.0,
                    },
                    "folds": [
                        {
                            "holdout_run": "tokyo_run1",
                            "holdout_selected_score_delta_distance_m": net_m - 1.0,
                            "holdout_selected_loss_distance_m": -2.0,
                            "holdout_selected_segments": 3,
                        },
                        {
                            "holdout_run": "nagoya_run1",
                            "holdout_selected_score_delta_distance_m": 1.0,
                            "holdout_selected_loss_distance_m": 0.0,
                            "holdout_selected_segments": 1,
                        },
                    ],
                }

            net_summary.write_text(json.dumps(summary(10.0, 80.0, 2)), encoding="utf-8")
            robust_summary.write_text(json.dumps(summary(18.0, 95.0, 2)), encoding="utf-8")

            argv = [
                "generate_ppc_selector_validation_scorecard.py",
                "--summary",
                f"net={net_summary}",
                "--summary",
                f"robust={robust_summary}",
                "--best-label",
                "robust",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_selector_scorecard.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (1400, 760))
            except ModuleNotFoundError:
                pass

    def test_ppc_rtk_trajectory_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_trajectory_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            output_png = temp_root / "ppc_rtk_trajectory.png"

            rows = [
                (2000, 0.0, 35.0000000, 139.0000000, 10.0),
                (2000, 1.0, 35.0000100, 139.0000100, 10.2),
                (2000, 2.0, 35.0000200, 139.0000200, 10.4),
                (2000, 3.0, 35.0000300, 139.0000300, 10.6),
            ]
            self.write_reference_csv(reference_csv, rows)
            self.write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 4),
                    (2000, 1.0, 35.0000101, 139.0000101, 10.2, 4),
                    (2000, 2.0, 35.0000202, 139.0000202, 10.5, 3),
                    (2000, 3.0, 35.0000300, 139.0000300, 10.6, 4),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0000000, 139.0000000, 10.0, 1),
                    (2000, 1.0, 35.0000110, 139.0000110, 10.5, 2),
                    (2000, 2.0, 35.0000200, 139.0000200, 10.4, 5),
                    (2000, 3.0, 35.0000300, 139.0000300, 10.6, 1),
                ],
            )

            argv = [
                "generate_ppc_rtk_trajectory.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(output_png),
                "--title",
                "Synthetic PPC trajectory",
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_trajectory.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)

            official_png = temp_root / "ppc_rtk_trajectory_official.png"
            bad_segments_json = temp_root / "coverage_quality.json"
            bad_segments_json.write_text(
                json.dumps(
                    {
                        "bad_segments": [
                            {
                                "start_tow_s": 1.0,
                                "end_tow_s": 2.0,
                                "epochs": 2,
                                "max_h_m": 3.2,
                            }
                        ]
                    }
                ),
                encoding="utf-8",
            )
            argv = [
                "generate_ppc_rtk_trajectory.py",
                "--lib-pos",
                str(lib_pos),
                "--rtklib-pos",
                str(rtklib_pos),
                "--reference-csv",
                str(reference_csv),
                "--output",
                str(official_png),
                "--title",
                "Synthetic PPC official trajectory",
                "--color-mode",
                "official",
                "--bad-segments-json",
                str(bad_segments_json),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_rtk_trajectory.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(official_png.exists())
            self.assertGreater(official_png.stat().st_size, 0)

    def test_architecture_diagram_main_renders_png(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_architecture_card_test_") as temp_dir:
            output_png = Path(temp_dir) / "architecture.png"

            argv = [
                "generate_architecture_diagram.py",
                "--output",
                str(output_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    architecture_diagram.main()

            self.assertTrue(output_png.exists())
            self.assertGreater(output_png.stat().st_size, 0)
            try:
                from PIL import Image

                with Image.open(output_png) as image:
                    self.assertEqual(image.size, (2240, 1408))
            except ModuleNotFoundError:
                pass


class SegmentedBenchmarkTest(unittest.TestCase):
    def write_reference_csv(self, path: Path, tows: list[float]) -> None:
        with path.open("w", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["gps_tow_s", "lat_deg", "lon_deg", "height_m"])
            for tow in tows:
                writer.writerow([tow, 0.0, 0.0, 0.0])

    def test_segmented_merge_uses_raw_child_solves(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_test_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            self.write_reference_csv(reference_csv, [0.0, 1.0, 2.0, 3.0, 4.0])

            args = argparse.Namespace(
                reference_csv=reference_csv,
                segment_epochs=2,
                warmup_epochs=1,
                jobs=1,
                rover=temp_root / "rover.obs",
                base=temp_root / "base.obs",
                nav=temp_root / "base.nav",
                lib_pos=temp_root / "merged.pos",
                lib_kml=temp_root / "merged.kml",
                mode="kinematic",
                glonass_ar="autocal",
                scorecard_title="Test Title",
            )
            dispatcher = temp_root / "dispatcher.py"
            dispatcher.write_text("#!/usr/bin/env python3\n", encoding="ascii")

            solve_commands: list[list[str]] = []
            pos2kml_commands: list[list[str]] = []

            def fake_subprocess_run(command: list[str], check: bool) -> None:
                self.assertTrue(check)
                solve_commands.append(command)
                out_path = Path(command[command.index("--out") + 1])
                skip_epochs = int(command[command.index("--skip-epochs") + 1])
                max_epochs = int(command[command.index("--max-epochs") + 1])
                end_epoch = min(skip_epochs + max_epochs, 5)
                with out_path.open("w", encoding="ascii") as handle:
                    handle.write("% synthetic segment\n")
                    for tow in range(skip_epochs, end_epoch):
                        handle.write(
                            f"2000 {float(tow):.1f} 0 0 0 0 0 0 2 8 1.0\n"
                        )

            def fake_run_command(command: list[str]) -> None:
                pos2kml_commands.append(command)

            with mock.patch.object(benchmark.subprocess, "run", side_effect=fake_subprocess_run):
                with mock.patch.object(benchmark, "run_command", side_effect=fake_run_command):
                    benchmark.run_segmented_lib_solve(args, dispatcher)

            self.assertEqual(len(solve_commands), 3)
            for command in solve_commands:
                self.assertIn("--no-kinematic-post-filter", command)

            self.assertEqual(len(pos2kml_commands), 1)
            self.assertEqual(pos2kml_commands[0][1], str(dispatcher))
            self.assertEqual(pos2kml_commands[0][2], "pos2kml")

            merged_lines = [
                line.strip()
                for line in args.lib_pos.read_text(encoding="ascii").splitlines()
                if line.strip() and not line.startswith("%")
            ]
            self.assertEqual(len(merged_lines), 5)
            merged_tows = [float(line.split()[1]) for line in merged_lines]
            self.assertEqual(merged_tows, [0.0, 1.0, 2.0, 3.0, 4.0])

    def make_benchmark_args(self, temp_root: Path, **overrides: object) -> argparse.Namespace:
        paths = {
            "rover": temp_root / "rover.obs",
            "base": temp_root / "base.obs",
            "nav": temp_root / "base.nav",
            "reference_csv": temp_root / "reference.csv",
            "rtklib_config": temp_root / "rtklib.conf",
            "rtklib_bin": temp_root / "rnx2rtkp",
            "malib_config": temp_root / "malib.conf",
            "malib_bin": None,
            "lib_pos": temp_root / "lib.pos",
            "lib_kml": temp_root / "lib.kml",
            "rtklib_pos": temp_root / "rtklib.pos",
            "malib_pos": temp_root / "malib.pos",
            "comparison_png": temp_root / "comparison.png",
            "scorecard_png": temp_root / "scorecard.png",
            "social_card_png": temp_root / "social_card.png",
            "summary_json": temp_root / "summary.json",
        }
        for key, path in paths.items():
            if isinstance(path, Path):
                if key == "malib_pos":
                    continue
                path.write_text("synthetic\n", encoding="ascii")
        defaults: dict[str, object] = {
            **paths,
            "comparison_title": "Comparison",
            "scorecard_title": "Scorecard",
            "social_card_title": "Social Card",
            "require_all_epochs_min": 0,
            "require_common_epoch_pairs_min": 0,
            "require_lib_all_p95_h_max": None,
            "require_lib_common_median_h_max": None,
            "require_lib_common_p95_h_max": None,
            "mode": "kinematic",
            "glonass_ar": "autocal",
            "skip_epochs": 0,
            "max_epochs": -1,
            "segment_epochs": 0,
            "warmup_epochs": 300,
            "jobs": 4,
        }
        defaults.update(overrides)
        return argparse.Namespace(**defaults)

    def test_main_partial_window_skips_segmented_and_downstream_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                skip_epochs=12,
                max_epochs=34,
                segment_epochs=100,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(
                        benchmark, "run_command", side_effect=commands.append
                    ):
                        exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_not_called()
            self.assertEqual(len(commands), 1)
            solve_command = commands[0]
            self.assertEqual(solve_command[2], "solve")
            self.assertIn("--no-kml", solve_command)
            self.assertIn("--skip-epochs", solve_command)
            self.assertIn("--max-epochs", solve_command)

    def test_main_full_segmented_run_uses_segment_solver_then_reports(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_main_") as temp_dir:
            temp_root = Path(temp_dir)
            args = self.make_benchmark_args(
                temp_root,
                segment_epochs=500,
                warmup_epochs=50,
                jobs=2,
            )

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "run_segmented_lib_solve") as segmented:
                    with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                        with mock.patch.object(
                            benchmark, "enforce_summary_requirements"
                        ) as summary_checks:
                            with mock.patch.object(
                                benchmark, "run_command", side_effect=commands.append
                            ):
                                exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            segmented.assert_called_once()
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 4)
            self.assertEqual(commands[0][0], str(args.rtklib_bin))
            self.assertEqual(commands[1][2], "driving-compare")
            self.assertEqual(commands[2][2], "scorecard")
            self.assertEqual(commands[3][2], "social-card")

    def test_write_summary_json_exports_all_and_common_epoch_metrics(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "custom_reference.csv"
            lib_pos = temp_root / "custom_lib.pos"
            rtklib_pos = temp_root / "custom_rtklib.pos"
            malib_pos = temp_root / "custom_malib.pos"
            summary_json = temp_root / "custom_summary.json"

            rows = [
                (2000, 0.0, 35.0, 139.0, 10.0),
                (2000, 1.0, 35.00001, 139.00001, 10.1),
                (2000, 2.0, 35.00002, 139.00002, 10.2),
            ]
            ScorecardRenderTest().write_reference_csv(reference_csv, rows)
            ScorecardRenderTest().write_lib_pos(
                lib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 4),
                    (2000, 1.0, 35.0000101, 139.0000101, 10.15, 3),
                    (2000, 2.0, 35.0000202, 139.0000202, 10.25, 4),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                rtklib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000102, 139.0000102, 10.12, 2),
                    (2000, 2.0, 35.0000203, 139.0000203, 10.22, 1),
                ],
            )
            ScorecardRenderTest().write_rtklib_pos(
                malib_pos,
                [
                    (2000, 0.0, 35.0, 139.0, 10.0, 1),
                    (2000, 1.0, 35.0000100, 139.0000100, 10.10, 1),
                    (2000, 2.0, 35.0000201, 139.0000201, 10.20, 2),
                ],
            )

            args = self.make_benchmark_args(
                temp_root,
                reference_csv=reference_csv,
                lib_pos=lib_pos,
                rtklib_pos=rtklib_pos,
                malib_pos=malib_pos,
                summary_json=summary_json,
            )

            returned_payload = benchmark.write_summary_json(args)

            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["dataset"], "UrbanNav Tokyo Odaiba")
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertEqual(payload["libgnss_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["rtklib_all_epochs"]["epochs"], 3)
            self.assertEqual(payload["malib_all_epochs"]["epochs"], 3)
            self.assertIn("median_h_m", payload["libgnss_common_epochs"])
            self.assertIn("p95_h_m", payload["rtklib_common_epochs"])
            self.assertIn("median_h_m", payload["malib_common_epochs"])
            self.assertEqual(payload["malib_common_epoch_pairs"], 3)
            self.assertEqual(payload, returned_payload)

    def test_main_full_run_invokes_optional_malib_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_benchmark_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            malib_bin = temp_root / "malib_rnx2rtkp"
            malib_bin.write_text("synthetic\n", encoding="ascii")
            args = self.make_benchmark_args(temp_root, malib_bin=malib_bin)

            commands: list[list[str]] = []
            with mock.patch.object(benchmark, "parse_args", return_value=args):
                with mock.patch.object(benchmark, "write_summary_json") as summary_writer:
                    with mock.patch.object(
                        benchmark, "enforce_summary_requirements"
                    ) as summary_checks:
                        with mock.patch.object(
                            benchmark, "run_command", side_effect=commands.append
                        ):
                            exit_code = benchmark.main()

            self.assertEqual(exit_code, 0)
            summary_writer.assert_called_once_with(args)
            summary_checks.assert_called_once_with(summary_writer.return_value, args)
            self.assertEqual(len(commands), 6)
            self.assertEqual(commands[0][2], "solve")
            self.assertEqual(commands[1][0], str(args.rtklib_bin))
            self.assertEqual(commands[2][0], str(args.malib_bin))
            self.assertEqual(commands[3][2], "driving-compare")
            self.assertEqual(commands[4][2], "scorecard")
            self.assertEqual(commands[5][2], "social-card")

    def test_enforce_summary_requirements_passes_and_fails(self) -> None:
        payload = {
            "common_epoch_pairs": 8123,
            "libgnss_all_epochs": {"epochs": 11637, "p95_h_m": 7.583936},
            "libgnss_common_epochs": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
        }
        passing_args = argparse.Namespace(
            require_all_epochs_min=11000,
            require_common_epoch_pairs_min=8000,
            require_lib_all_p95_h_max=8.0,
            require_lib_common_median_h_max=0.8,
            require_lib_common_p95_h_max=6.5,
        )
        benchmark.enforce_summary_requirements(payload, passing_args)

        failing_args = argparse.Namespace(
            require_all_epochs_min=12000,
            require_common_epoch_pairs_min=9000,
            require_lib_all_p95_h_max=7.0,
            require_lib_common_median_h_max=0.7,
            require_lib_common_p95_h_max=5.0,
        )
        with self.assertRaises(SystemExit) as context:
            benchmark.enforce_summary_requirements(payload, failing_args)

        message = str(context.exception)
        self.assertIn("all-epoch matched count", message)
        self.assertIn("common epoch pairs", message)
        self.assertIn("all-epoch p95_h", message)
        self.assertIn("common-epoch median_h", message)
        self.assertIn("common-epoch p95_h", message)


class ShortBaselineSignoffTest(unittest.TestCase):
    def write_rinex_header(self, path: Path, position: tuple[float, float, float]) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("     3.02           OBSERVATION DATA    M                   RINEX VERSION / TYPE\n")
            handle.write(
                f"{position[0]:14.4f}{position[1]:14.4f}{position[2]:14.4f}"
                "                  APPROX POSITION XYZ\n"
            )
            handle.write("                                                            END OF HEADER\n")

    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic short-baseline signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_short_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            rover = temp_root / "rover.rnx"
            base = temp_root / "base.rnx"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3957184.1109, 3310231.7255, 3737703.9594)
            base_position = (-3957199.2400, 3310199.6680, 3737711.7080)

            self.write_rinex_header(rover, rover_position)
            self.write_rinex_header(base, base_position)
            nav.write_text("synthetic\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (2000, 0.0, rover_position, 4, 15),
                    (2000, 1.0, (-3957184.0109, 3310231.7255, 3737703.9594), 4, 14),
                    (2000, 2.0, (-3957184.2109, 3310231.7255, 3737703.9594), 3, 14),
                ],
            )

            args = argparse.Namespace(
                rover=rover,
                base=base,
                nav=nav,
                out=out,
                summary_json=summary_json,
                require_fix_rate_min=60.0,
                require_mean_error_max=0.2,
                require_max_error_max=0.25,
                require_mean_sats_min=14.0,
            )

            payload = short_signoff.build_summary_payload(args)

            self.assertEqual(payload["dataset"], "Tsukuba short_baseline")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertAlmostEqual(payload["fix_rate_pct"], 66.666667, places=5)
            self.assertAlmostEqual(payload["mean_position_error_m"], 0.066667, places=6)
            self.assertAlmostEqual(payload["max_position_error_m"], 0.1, places=6)
            self.assertAlmostEqual(payload["mean_satellites"], 14.333333, places=5)
            self.assertTrue(summary_json.exists())

            short_signoff.enforce_summary_requirements(payload, args)

            failing_args = argparse.Namespace(
                require_fix_rate_min=90.0,
                require_mean_error_max=0.05,
                require_max_error_max=0.05,
                require_mean_sats_min=15.0,
            )
            with self.assertRaises(SystemExit) as context:
                short_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("fix rate", message)
            self.assertIn("mean position error", message)
            self.assertIn("max position error", message)
        self.assertIn("mean satellites", message)


class PPPStaticSignoffTest(unittest.TestCase):
    def write_rinex_header(self, path: Path, position: tuple[float, float, float]) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("     2.10           OBSERVATION DATA    G                   RINEX VERSION / TYPE\n")
            handle.write(
                f"{position[0]:14.4f}{position[1]:14.4f}{position[2]:14.4f}"
                "                  APPROX POSITION XYZ\n"
            )
            handle.write("                                                            END OF HEADER\n")

    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppp static signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3978242.4348, 3382841.1715, 3649902.7667)

            self.write_rinex_header(obs, rover_position)
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, rover_position, 5, 9),
                    (1316, 518430.0, (-3978243.2000, 3382840.6000, 3649902.4000), 5, 8),
                    (1316, 518460.0, (-3978243.7000, 3382840.2000, 3649902.1000), 1, 9),
                ],
            )

            args = argparse.Namespace(
                obs=obs,
                nav=nav,
                sp3=None,
                clk=None,
                enable_ar=False,
                ar_ratio_threshold=3.0,
                generate_products=False,
                out=out,
                summary_json=summary_json,
                require_valid_epochs_min=3,
                require_mean_error_max=2.0,
                require_max_error_max=2.5,
                require_mean_sats_min=8.0,
                require_ppp_solution_rate_min=60.0,
                require_ppp_fixed_epochs_min=None,
                require_converged=True,
                require_convergence_time_max=120.0,
                require_ionex_corrections_min=1,
                require_dcb_corrections_min=1,
                malib_pos=None,
            )
            args.ppp_run_summary = {
                "converged": True,
                "convergence_time_s": 45.0,
                "ionex_corrections": 12,
                "ionex_meters": 23.0,
                "dcb_corrections": 4,
                "dcb_meters": 1.2,
            }

            payload = ppp_static_signoff.build_summary_payload(args)
            ppp_static_signoff.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "sample static PPP")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["ppp_float_epochs"], 2)
            self.assertEqual(payload["ppp_fixed_epochs"], 0)
            self.assertEqual(payload["fallback_epochs"], 1)
            self.assertGreaterEqual(payload["ppp_solution_rate_pct"], 60.0)
            self.assertLessEqual(payload["mean_position_error_m"], 2.0)
            self.assertLessEqual(payload["max_position_error_m"], 2.5)
            self.assertGreaterEqual(payload["mean_satellites"], 8.0)
            self.assertTrue(payload["ppp_converged"])
            self.assertEqual(payload["ionex_corrections"], 12)
            self.assertEqual(payload["dcb_corrections"], 4)

            failing_args = argparse.Namespace(
                require_valid_epochs_min=4,
                require_mean_error_max=0.1,
                require_max_error_max=0.2,
                require_mean_sats_min=10.0,
                require_ppp_solution_rate_min=90.0,
                require_ppp_fixed_epochs_min=1,
                require_converged=True,
                require_convergence_time_max=10.0,
                require_ionex_corrections_min=20,
                require_dcb_corrections_min=5,
            )
            with self.assertRaises(SystemExit) as context:
                ppp_static_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("valid epochs", message)
            self.assertIn("mean position error", message)
            self.assertIn("max position error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("PPP solution rate", message)
            self.assertIn("PPP fixed epochs", message)
            self.assertIn("convergence time", message)
            self.assertIn("IONEX corrections", message)
            self.assertIn("DCB corrections", message)

    def test_build_summary_payload_with_malib_sidecar(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_static_signoff_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "solution.pos"
            malib_pos = temp_root / "malib.pos"
            summary_json = temp_root / "summary.json"
            rover_position = (-3978242.4348, 3382841.1715, 3649902.7667)

            self.write_rinex_header(obs, rover_position)
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, rover_position, 5, 9),
                    (1316, 518430.0, rover_position, 5, 8),
                ],
            )
            malib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic malib xyz",
                        "2005/04/02 00:00:00.000 -3978242.4348 3382841.1715 3649902.7667 6 8",
                        "2005/04/02 00:00:30.000 -3978243.4348 3382841.1715 3649902.7667 6 7",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            args = argparse.Namespace(
                obs=obs,
                nav=nav,
                sp3=None,
                clk=None,
                enable_ar=False,
                ar_ratio_threshold=3.0,
                generate_products=False,
                out=out,
                malib_pos=malib_pos,
                summary_json=summary_json,
                require_valid_epochs_min=None,
                require_mean_error_max=None,
                require_max_error_max=None,
                require_mean_sats_min=None,
                require_ppp_solution_rate_min=None,
                require_ppp_fixed_epochs_min=None,
            )

            payload = ppp_static_signoff.build_summary_payload(args)
            self.assertEqual(payload["malib_epochs"], 2)
            self.assertAlmostEqual(payload["malib_mean_position_error_m"], 0.5)
            self.assertIn("libgnss_minus_malib_mean_error_m", payload)


class PPPKinematicSignoffTest(unittest.TestCase):
    def write_pos(
        self,
        path: Path,
        records: list[tuple[int, float, tuple[float, float, float], int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppp kinematic signoff\n")
            for week, tow, position, status, satellites in records:
                handle.write(
                    f"{week} {tow:.1f} {position[0]:.4f} {position[1]:.4f} {position[2]:.4f} "
                    f"35.0 139.0 10.0 {status} {satellites} 1.0\n"
                )

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            base = temp_root / "base.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "ppp_solution.pos"
            reference_pos = temp_root / "reference.pos"
            summary_json = temp_root / "summary.json"

            obs.write_text("synthetic obs\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, (-3978242.0, 3382841.0, 3649903.0), 5, 20),
                    (1316, 518430.0, (-3978248.0, 3382839.0, 3649900.0), 5, 21),
                    (1316, 518460.0, (-3978255.0, 3382834.0, 3649898.0), 1, 19),
                ],
            )
            self.write_pos(
                reference_pos,
                [
                    (1316, 518400.0, (-3978240.0, 3382840.0, 3649902.0), 4, 20),
                    (1316, 518430.0, (-3978244.0, 3382840.0, 3649900.0), 4, 21),
                    (1316, 518460.0, (-3978250.0, 3382835.0, 3649897.0), 3, 19),
                ],
            )

            args = argparse.Namespace(
                obs=obs,
                base=base,
                nav=nav,
                out=out,
                reference_pos=reference_pos,
                summary_json=summary_json,
                require_common_epoch_pairs_min=3,
                require_reference_fix_rate_min=60.0,
                require_mean_error_max=6.0,
                require_p95_error_max=8.0,
                require_max_error_max=8.0,
                require_mean_sats_min=19.0,
                require_ppp_solution_rate_min=60.0,
                require_converged=True,
                require_convergence_time_max=120.0,
                require_ionex_corrections_min=2,
                require_dcb_corrections_min=1,
                malib_pos=None,
            )
            args.ppp_run_summary = {
                "converged": True,
                "convergence_time_s": 60.0,
                "ionex_corrections": 8,
                "ionex_meters": 15.0,
                "dcb_corrections": 3,
                "dcb_meters": 0.8,
            }

            payload = ppp_kinematic_signoff.build_summary_payload(args)
            ppp_kinematic_signoff.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "sample kinematic PPP")
            self.assertEqual(payload["epochs"], 3)
            self.assertEqual(payload["common_epoch_pairs"], 3)
            self.assertEqual(payload["ppp_float_epochs"], 2)
            self.assertEqual(payload["ppp_fixed_epochs"], 0)
            self.assertEqual(payload["fallback_epochs"], 1)
            self.assertGreaterEqual(payload["reference_fix_rate_pct"], 60.0)
            self.assertLessEqual(payload["mean_position_error_m"], 6.0)
            self.assertLessEqual(payload["p95_position_error_m"], 8.0)
            self.assertLessEqual(payload["max_position_error_m"], 8.0)
            self.assertGreaterEqual(payload["mean_satellites"], 19.0)
            self.assertGreaterEqual(payload["ppp_solution_rate_pct"], 60.0)
            self.assertTrue(payload["ppp_converged"])
            self.assertEqual(payload["ionex_corrections"], 8)
            self.assertEqual(payload["dcb_corrections"], 3)

            failing_args = argparse.Namespace(
                require_common_epoch_pairs_min=4,
                require_reference_fix_rate_min=90.0,
                require_mean_error_max=1.0,
                require_p95_error_max=2.0,
                require_max_error_max=3.0,
                require_mean_sats_min=25.0,
                require_ppp_solution_rate_min=90.0,
                require_converged=True,
                require_convergence_time_max=10.0,
                require_ionex_corrections_min=20,
                require_dcb_corrections_min=4,
            )
            with self.assertRaises(SystemExit) as context:
                ppp_kinematic_signoff.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("common epoch pairs", message)
            self.assertIn("reference fix rate", message)
            self.assertIn("mean position error", message)
            self.assertIn("p95 position error", message)
            self.assertIn("max position error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("PPP solution rate", message)
            self.assertIn("convergence time", message)
            self.assertIn("IONEX corrections", message)
            self.assertIn("DCB corrections", message)

    def test_build_summary_payload_with_malib_sidecar(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_kinematic_signoff_malib_") as temp_dir:
            temp_root = Path(temp_dir)
            obs = temp_root / "rover.obs"
            base = temp_root / "base.obs"
            nav = temp_root / "nav.rnx"
            out = temp_root / "ppp_solution.pos"
            reference_pos = temp_root / "reference.pos"
            malib_pos = temp_root / "malib.pos"
            summary_json = temp_root / "summary.json"

            obs.write_text("synthetic obs\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_pos(
                out,
                [
                    (1316, 518400.0, (-3978242.0, 3382841.0, 3649903.0), 5, 20),
                    (1316, 518430.0, (-3978248.0, 3382839.0, 3649900.0), 5, 21),
                ],
            )
            self.write_pos(
                reference_pos,
                [
                    (1316, 518400.0, (-3978240.0, 3382840.0, 3649902.0), 4, 20),
                    (1316, 518430.0, (-3978244.0, 3382840.0, 3649900.0), 4, 21),
                ],
            )
            malib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic malib xyz",
                        "2005/04/02 00:00:00.000 -3978240.5000 3382840.0000 3649902.0000 6 7",
                        "2005/04/02 00:00:30.000 -3978244.5000 3382840.0000 3649900.0000 6 6",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            args = argparse.Namespace(
                obs=obs,
                base=base,
                nav=nav,
                out=out,
                reference_pos=reference_pos,
                malib_pos=malib_pos,
                summary_json=summary_json,
                require_common_epoch_pairs_min=None,
                require_reference_fix_rate_min=None,
                require_mean_error_max=None,
                require_p95_error_max=None,
                require_max_error_max=None,
                require_mean_sats_min=None,
                require_ppp_solution_rate_min=None,
            )

            payload = ppp_kinematic_signoff.build_summary_payload(args)
            self.assertEqual(payload["malib_common_epoch_pairs"], 2)
            self.assertAlmostEqual(payload["malib_mean_position_error_m"], 0.5)
            self.assertIn("libgnss_minus_malib_p95_error_m", payload)


class LiveSignoffTest(unittest.TestCase):
    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_json = temp_root / "summary.json"
            args = argparse.Namespace(
                out=temp_root / "live.pos",
                summary_json=summary_json,
                log_out=temp_root / "live.log",
                use_existing_log=None,
                require_termination="completed",
                require_aligned_epochs_min=3,
                require_written_solutions_min=3,
                require_fixed_solutions_min=1,
                require_rover_decoder_errors_max=0,
                require_base_decoder_errors_max=0,
                require_realtime_factor_min=1.0,
                require_effective_epoch_rate_min=10.0,
                require_solver_wall_time_max=2.0,
            )
            summary_line = (
                "summary: termination=completed aligned_epochs=3 written_solutions=3 "
                "fixed_solutions=1 rover_decoder_errors=0 base_decoder_errors=0 "
                "solver_wall_time_s=0.250000 solution_span_s=1.000000 "
                "realtime_factor=4.000000 effective_epoch_rate_hz=12.000000"
            )

            payload = live_signoff.build_summary_payload(
                args,
                summary_line,
                "stdout text\n" + summary_line + "\n",
                "",
                0,
            )
            self.assertEqual(payload["metrics"]["termination"], "completed")
            self.assertEqual(payload["metrics"]["written_solutions"], 3)
            self.assertTrue(summary_json.exists())
            live_signoff.enforce_requirements(payload, args)

            failing_args = argparse.Namespace(
                **{
                    **vars(args),
                    "require_realtime_factor_min": 5.0,
                    "require_written_solutions_min": 4,
                }
            )
            with self.assertRaises(SystemExit) as ctx:
                live_signoff.enforce_requirements(payload, failing_args)

            self.assertIn("realtime_factor", str(ctx.exception))
            self.assertIn("written_solutions", str(ctx.exception))


class PPCDemoTest(unittest.TestCase):
    def write_reference_csv(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float]],
    ) -> None:
        with path.open("w", newline="", encoding="ascii") as handle:
            writer = csv.writer(handle)
            writer.writerow(
                [
                    "gps_week",
                    "gps_tow_s",
                    "lat_deg",
                    "lon_deg",
                    "height_m",
                    "roll_deg",
                    "pitch_deg",
                    "yaw_deg",
                ]
            )
            for week, tow, lat, lon, height in rows:
                writer.writerow([week, f"{tow:.3f}", lat, lon, height, 0.0, 0.0, 0.0])

    def write_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic ppc demo solution\n")
            for week, tow, lat, lon, height, status, satellites in rows:
                ecef = comparison.llh_to_ecef(lat, lon, height)
                handle.write(
                    f"{week} {tow:.3f} {ecef[0]:.6f} {ecef[1]:.6f} {ecef[2]:.6f} "
                    f"{lat:.9f} {lon:.9f} {height:.4f} {status} {satellites} 1.0\n"
                )

    def write_rtklib_pos(
        self,
        path: Path,
        rows: list[tuple[int, float, float, float, float, int]],
    ) -> None:
        with path.open("w", encoding="ascii") as handle:
            handle.write("% synthetic rtklib solution\n")
            for week, tow, lat, lon, height, quality in rows:
                stamp = ppc_demo.GPS_EPOCH + timedelta(weeks=week, seconds=tow)
                handle.write(
                    f"{stamp:%Y/%m/%d %H:%M:%S.%f}"[:-3]
                    + f" {lat:.9f} {lon:.9f} {height:.4f} {quality} 0 0 0 0 0 0\n"
                )

    def test_run_solver_passes_rtk_iono_option(self) -> None:
        args = argparse.Namespace(
            solver="rtk",
            preset="low-cost",
            iono="iflc",
            ratio=2.4,
            max_hold_div=5.0,
            max_pos_jump=20.0,
            max_pos_jump_min=20.0,
            max_pos_jump_rate=25.0,
            max_float_spp_div=30.0,
            max_float_prefit_rms=6.0,
            max_float_prefit_max=30.0,
            max_float_prefit_reset_streak=5,
            min_float_prefit_trusted_jump=8.0,
            max_update_nis_per_obs=12.0,
            max_consec_float_reset=10,
            max_consec_nonfix_reset=10,
            max_postfix_rms=0.20,
            enable_wide_lane_ar=True,
            wide_lane_threshold=0.10,
            fixed_bridge_burst_guard=True,
            fixed_bridge_burst_max_anchor_gap=30.0,
            fixed_bridge_burst_min_boundary_gap=1.0,
            fixed_bridge_burst_max_residual=20.0,
            fixed_bridge_burst_max_segment_epochs=12,
            arfilter=False,
            arfilter_margin=None,
            min_hold_count=None,
            hold_ratio_threshold=None,
            no_kinematic_post_filter=True,
            no_nonfix_drift_guard=False,
            nonfix_drift_max_anchor_gap=None,
            nonfix_drift_max_anchor_speed=None,
            nonfix_drift_max_residual=None,
            nonfix_drift_min_horizontal_residual=None,
            nonfix_drift_min_segment_epochs=None,
            nonfix_drift_max_segment_epochs=None,
            no_spp_height_step_guard=False,
            spp_height_step_min=None,
            spp_height_step_rate=None,
            float_bridge_tail_guard=True,
            float_bridge_tail_max_anchor_gap=None,
            float_bridge_tail_min_anchor_speed=None,
            float_bridge_tail_max_anchor_speed=None,
            float_bridge_tail_max_residual=None,
            float_bridge_tail_min_segment_epochs=None,
            max_epochs=120,
        )
        commands: list[list[str]] = []

        with mock.patch.object(ppc_demo, "run_command", side_effect=commands.append):
            elapsed = ppc_demo.run_solver(
                args,
                Path("rover.obs"),
                Path("base.obs"),
                Path("base.nav"),
                Path("out.pos"),
            )

        self.assertGreaterEqual(elapsed, 0.0)
        self.assertEqual(len(commands), 1)
        self.assertIn("--iono", commands[0])
        self.assertIn("iflc", commands[0])
        self.assertIn("--ratio", commands[0])
        self.assertIn("2.4", commands[0])
        self.assertIn("--max-hold-div", commands[0])
        self.assertIn("5.0", commands[0])
        self.assertIn("--max-pos-jump", commands[0])
        self.assertIn("20.0", commands[0])
        self.assertIn("--max-pos-jump-min", commands[0])
        self.assertIn("20.0", commands[0])
        self.assertIn("--max-pos-jump-rate", commands[0])
        self.assertIn("25.0", commands[0])
        self.assertIn("--max-float-spp-div", commands[0])
        self.assertIn("30.0", commands[0])
        self.assertIn("--max-float-prefit-rms", commands[0])
        self.assertIn("6.0", commands[0])
        self.assertIn("--max-float-prefit-max", commands[0])
        self.assertIn("30.0", commands[0])
        self.assertIn("--max-float-prefit-reset-streak", commands[0])
        self.assertIn("5", commands[0])
        self.assertIn("--min-float-prefit-trusted-jump", commands[0])
        self.assertIn("8.0", commands[0])
        self.assertIn("--max-update-nis-per-obs", commands[0])
        self.assertIn("12.0", commands[0])
        self.assertIn("--max-consec-float-reset", commands[0])
        self.assertIn("10", commands[0])
        self.assertIn("--max-consec-nonfix-reset", commands[0])
        self.assertIn("--max-postfix-rms", commands[0])
        self.assertIn("0.2", commands[0])
        self.assertIn("--enable-wide-lane-ar", commands[0])
        self.assertIn("--wide-lane-threshold", commands[0])
        self.assertIn("0.1", commands[0])
        self.assertIn("--fixed-bridge-burst-guard", commands[0])
        self.assertIn("--fixed-bridge-burst-max-residual", commands[0])
        self.assertIn("20.0", commands[0])
        self.assertIn("--no-arfilter", commands[0])
        self.assertIn("--no-kinematic-post-filter", commands[0])

    def test_build_summary_payload_and_requirements(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_demo_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "tokyo" / "run1"
            run_dir.mkdir(parents=True)
            rover = run_dir / "rover.obs"
            base = run_dir / "base.obs"
            nav = run_dir / "base.nav"
            reference_csv = run_dir / "reference.csv"
            out = temp_root / "ppc_demo.pos"
            rtklib_pos = temp_root / "ppc_demo_rtklib.pos"
            commercial_pos = temp_root / "commercial_receiver.csv"
            commercial_matches = temp_root / "commercial_receiver_matches.csv"
            summary_json = temp_root / "ppc_demo_summary.json"

            rover.write_text("synthetic rover\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            self.write_reference_csv(
                reference_csv,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                    (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                    (2300, 1000.4, 35.1000200, 139.1000400, 42.4),
                ],
            )
            self.write_pos(
                out,
                [
                    (2300, 1000.0, 35.1000002, 139.1000001, 42.1, 4, 12),
                    (2300, 1000.2, 35.1000099, 139.1000202, 42.3, 4, 13),
                    (2300, 1000.4, 35.1000197, 139.1000398, 42.5, 3, 11),
                ],
            )
            self.write_rtklib_pos(
                rtklib_pos,
                [
                    (2300, 1000.0, 35.1000004, 139.1000003, 42.2, 1),
                    (2300, 1000.2, 35.1000103, 139.1000205, 42.4, 1),
                    (2300, 1000.4, 35.1000205, 139.1000404, 42.6, 2),
                ],
            )
            commercial_pos.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,solution_status,num_satellites",
                        "2300,1000.0,35.1000001,139.1000001,42.1,rtk_fixed,14",
                        "2300,1000.2,35.1000101,139.1000201,42.2,rtk_fixed,14",
                        "2300,1000.4,35.1000201,139.1000401,42.4,rtk_float,13",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            args = argparse.Namespace(
                dataset_root=None,
                city="tokyo",
                run="run1",
                run_dir=run_dir,
                solver="rtk",
                rover=rover,
                base=base,
                nav=nav,
                reference_csv=reference_csv,
                out=out,
                summary_json=summary_json,
                rtklib_pos=rtklib_pos,
                rtklib_bin=None,
                rtklib_config=None,
                use_existing_rtklib_solution=True,
                rtklib_solver_wall_time_s=0.1,
                commercial_pos=commercial_pos,
                commercial_rover=None,
                commercial_base=None,
                commercial_nav=None,
                commercial_out=None,
                use_existing_commercial_solution=False,
                commercial_format="auto",
                commercial_label="survey_receiver",
                commercial_matched_csv=commercial_matches,
                commercial_solver_wall_time_s=0.2,
                max_epochs=120,
                match_tolerance_s=0.25,
                use_existing_solution=True,
                solver_wall_time_s=0.5,
                sp3=None,
                clk=None,
                antex=None,
                blq=None,
                enable_ar=False,
                iono="iflc",
                ratio=2.4,
                max_hold_div=5.0,
                max_pos_jump=20.0,
                max_pos_jump_min=20.0,
                max_pos_jump_rate=25.0,
                max_float_spp_div=30.0,
                max_float_prefit_rms=6.0,
                max_float_prefit_max=30.0,
                max_float_prefit_reset_streak=5,
                min_float_prefit_trusted_jump=8.0,
                max_update_nis_per_obs=12.0,
                max_consec_float_reset=10,
                max_consec_nonfix_reset=10,
                max_postfix_rms=0.20,
                enable_wide_lane_ar=True,
                wide_lane_threshold=0.10,
                fixed_bridge_burst_guard=True,
                fixed_bridge_burst_max_anchor_gap=30.0,
                fixed_bridge_burst_min_boundary_gap=1.0,
                fixed_bridge_burst_max_residual=20.0,
                fixed_bridge_burst_max_segment_epochs=12,
                low_dynamics=False,
                no_kinematic_post_filter=True,
                no_spp_height_step_guard=False,
                spp_height_step_min=None,
                spp_height_step_rate=None,
                float_bridge_tail_guard=True,
                float_bridge_tail_max_anchor_gap=None,
                float_bridge_tail_min_anchor_speed=None,
                float_bridge_tail_max_anchor_speed=None,
                float_bridge_tail_max_residual=None,
                float_bridge_tail_min_segment_epochs=None,
                require_valid_epochs_min=3,
                require_matched_epochs_min=3,
                require_fix_rate_min=60.0,
                require_median_h_max=0.2,
                require_p95_h_max=0.2,
                require_max_h_max=0.2,
                require_p95_up_max=0.2,
                require_mean_sats_min=11.0,
                require_solver_wall_time_max=1.0,
                require_realtime_factor_min=0.5,
                require_effective_epoch_rate_min=5.0,
                require_lib_fix_rate_vs_rtklib_min_delta=0.0,
                require_lib_median_h_vs_rtklib_max_delta=0.0,
                require_lib_p95_h_vs_rtklib_max_delta=0.0,
                _dataset_city="tokyo",
                _dataset_run="run1",
            )

            payload = ppc_demo.build_summary_payload(
                args,
                run_dir,
                rover,
                base,
                nav,
                reference_csv,
                out,
                summary_json,
                solver_wall_time_s=args.solver_wall_time_s,
            )
            ppc_demo.enforce_summary_requirements(payload, args)

            self.assertEqual(payload["dataset"], "PPC-Dataset tokyo run1")
            self.assertEqual(payload["solver"], "rtk")
            self.assertEqual(payload["rtk_iono"], "iflc")
            self.assertEqual(payload["rtk_ratio_threshold"], 2.4)
            self.assertEqual(payload["rtk_max_hold_divergence_m"], 5.0)
            self.assertEqual(payload["rtk_max_position_jump_m"], 20.0)
            self.assertEqual(payload["rtk_max_position_jump_min_m"], 20.0)
            self.assertEqual(payload["rtk_max_position_jump_rate_mps"], 25.0)
            self.assertEqual(payload["rtk_max_float_spp_divergence_m"], 30.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_rms_m"], 6.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_max_m"], 30.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_reset_streak"], 5)
            self.assertEqual(payload["rtk_min_float_prefit_residual_trusted_jump_m"], 8.0)
            self.assertEqual(payload["rtk_max_update_nis_per_observation"], 12.0)
            self.assertEqual(payload["rtk_max_consecutive_float_for_reset"], 10)
            self.assertEqual(payload["rtk_max_consecutive_nonfix_for_reset"], 10)
            self.assertEqual(payload["rtk_max_postfix_residual_rms_m"], 0.20)
            self.assertTrue(payload["rtk_wide_lane_ar_enabled"])
            self.assertEqual(payload["rtk_wide_lane_threshold"], 0.10)
            self.assertTrue(payload["fixed_bridge_burst_guard_enabled"])
            self.assertEqual(payload["fixed_bridge_burst_guard"]["max_anchor_gap_s"], 30.0)
            self.assertEqual(payload["fixed_bridge_burst_guard"]["min_boundary_gap_s"], 1.0)
            self.assertEqual(payload["fixed_bridge_burst_guard"]["max_residual_m"], 20.0)
            self.assertEqual(payload["fixed_bridge_burst_guard"]["max_segment_epochs"], 12)
            self.assertEqual(payload["rtk_output_profile"], "coverage")
            self.assertFalse(payload["kinematic_post_filter_enabled"])
            self.assertTrue(payload["nonfix_drift_guard_enabled"])
            self.assertEqual(payload["nonfix_drift_guard"]["max_anchor_gap_s"], 120.0)
            self.assertEqual(payload["nonfix_drift_guard"]["max_anchor_speed_mps"], 1.0)
            self.assertEqual(payload["nonfix_drift_guard"]["max_residual_m"], 30.0)
            self.assertEqual(payload["nonfix_drift_guard"]["min_horizontal_residual_m"], 0.0)
            self.assertEqual(payload["nonfix_drift_guard"]["min_segment_epochs"], 20)
            self.assertEqual(payload["nonfix_drift_guard"]["max_segment_epochs"], 0)
            self.assertTrue(payload["spp_height_step_guard_enabled"])
            self.assertEqual(payload["spp_height_step_guard"]["min_step_m"], 30.0)
            self.assertEqual(payload["spp_height_step_guard"]["max_rate_mps"], 4.0)
            self.assertTrue(payload["float_bridge_tail_guard_enabled"])
            self.assertEqual(payload["float_bridge_tail_guard"]["max_anchor_gap_s"], 120.0)
            self.assertEqual(payload["float_bridge_tail_guard"]["min_anchor_speed_mps"], 0.4)
            self.assertEqual(payload["float_bridge_tail_guard"]["max_anchor_speed_mps"], 1.0)
            self.assertEqual(payload["float_bridge_tail_guard"]["max_residual_m"], 12.0)
            self.assertEqual(payload["float_bridge_tail_guard"]["min_segment_epochs"], 20)
            provenance = payload["receiver_observation_provenance"]
            self.assertEqual(provenance["vehicle_receiver"], "Septentrio mosaic-X5")
            self.assertEqual(provenance["vehicle_antenna"], "Trimble AT1675")
            self.assertEqual(provenance["reference_station_receiver"], "Trimble Alloy")
            self.assertFalse(provenance["receiver_engine_solution_available"])
            self.assertEqual(payload["valid_epochs"], 3)
            self.assertEqual(payload["matched_epochs"], 3)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertEqual(payload["positioning_rate_pct"], 100.0)
            self.assertGreaterEqual(payload["fix_rate_pct"], 60.0)
            self.assertEqual(payload["ppc_score_3d_50cm_epochs"], 3)
            self.assertEqual(payload["ppc_score_3d_50cm_matched_pct"], 100.0)
            self.assertEqual(payload["ppc_score_3d_50cm_ref_pct"], 100.0)
            self.assertEqual(payload["ppc_official_score_pct"], 100.0)
            self.assertGreater(payload["ppc_official_total_distance_m"], 0.0)
            self.assertEqual(
                payload["ppc_official_score_distance_m"],
                payload["ppc_official_total_distance_m"],
            )
            self.assertLessEqual(payload["median_h_m"], 0.2)
            self.assertLessEqual(payload["p95_h_m"], 0.2)
            self.assertLessEqual(payload["max_h_m"], 0.2)
            self.assertLessEqual(payload["p95_abs_up_m"], 0.2)
            self.assertGreaterEqual(payload["mean_satellites"], 11.0)
            self.assertEqual(payload["solver_wall_time_s"], 0.5)
            self.assertEqual(payload["solution_span_s"], 0.4)
            self.assertEqual(payload["realtime_factor"], 0.8)
            self.assertEqual(payload["effective_epoch_rate_hz"], 6.0)
            self.assertIn("rtklib", payload)
            self.assertEqual(payload["rtklib"]["matched_epochs"], 3)
            self.assertEqual(payload["rtklib"]["solver_wall_time_s"], 0.1)
            self.assertAlmostEqual(payload["rtklib"]["realtime_factor"], 4.0, places=5)
            self.assertIn("delta_vs_rtklib", payload)
            self.assertEqual(payload["delta_vs_rtklib"]["positioning_rate_pct"], 0.0)
            self.assertEqual(payload["delta_vs_rtklib"]["ppc_score_3d_50cm_ref_pct"], 0.0)
            self.assertEqual(payload["delta_vs_rtklib"]["ppc_official_score_pct"], 0.0)
            self.assertIn("commercial_receiver", payload)
            self.assertEqual(payload["commercial_receiver"]["label"], "survey_receiver")
            self.assertEqual(payload["commercial_receiver"]["matched_epochs"], 3)
            self.assertEqual(payload["commercial_receiver"]["fixed_epochs"], 2)
            self.assertEqual(payload["commercial_receiver"]["matched_csv"], str(commercial_matches))
            self.assertTrue(commercial_matches.exists())
            self.assertIn("delta_vs_commercial_receiver", payload)
            self.assertTrue(summary_json.exists())

            failing_args = argparse.Namespace(
                require_valid_epochs_min=4,
                require_matched_epochs_min=4,
                require_fix_rate_min=90.0,
                require_median_h_max=0.01,
                require_p95_h_max=0.01,
                require_max_h_max=0.01,
                require_p95_up_max=0.01,
                require_mean_sats_min=20.0,
                require_solver_wall_time_max=0.1,
                require_realtime_factor_min=2.0,
                require_effective_epoch_rate_min=10.0,
                require_lib_fix_rate_vs_rtklib_min_delta=10.0,
                require_lib_median_h_vs_rtklib_max_delta=-0.01,
                require_lib_p95_h_vs_rtklib_max_delta=-0.01,
            )
            with self.assertRaises(SystemExit) as context:
                ppc_demo.enforce_summary_requirements(payload, failing_args)

            message = str(context.exception)
            self.assertIn("valid epochs", message)
            self.assertIn("matched epochs", message)
            self.assertIn("fix rate", message)
            self.assertIn("median horizontal error", message)
            self.assertIn("p95 horizontal error", message)
            self.assertIn("max horizontal error", message)
            self.assertIn("p95 absolute up error", message)
            self.assertIn("mean satellites", message)
            self.assertIn("solver wall time", message)
            self.assertIn("realtime factor", message)
            self.assertIn("effective epoch rate", message)
            self.assertIn("RTKLIB", message)

    def test_run_rtklib_solver_executes_binary_path(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_rtklib_bin_") as temp_dir:
            temp_root = Path(temp_dir)
            run_dir = temp_root / "nagoya" / "run1"
            run_dir.mkdir(parents=True)
            rover = run_dir / "rover.obs"
            base = run_dir / "base.obs"
            nav = run_dir / "base.nav"
            reference_csv = run_dir / "reference.csv"
            rtklib_pos = temp_root / "rtklib.pos"
            config_path = temp_root / "rtklib.conf"
            fake_rtklib = temp_root / "fake_rnx2rtkp.py"

            rover.write_text("synthetic rover\n", encoding="ascii")
            base.write_text("synthetic base\n", encoding="ascii")
            nav.write_text("synthetic nav\n", encoding="ascii")
            config_path.write_text("pos1-navsys        =1\n", encoding="ascii")
            self.write_reference_csv(
                reference_csv,
                [
                    (2300, 1000.0, 35.1000000, 139.1000000, 42.0),
                    (2300, 1000.2, 35.1000100, 139.1000200, 42.2),
                ],
            )
            fake_rtklib.write_text(
                """#!/usr/bin/env python3
import sys
from pathlib import Path

args = sys.argv[1:]
out = Path(args[args.index("-o") + 1])
out.write_text(
    "% synthetic rtklib solution\\n"
    "2024/02/18 00:16:22.000 35.100000000 139.100000000 42.0000 1 0 0 0 0 0 0\\n"
    "2024/02/18 00:16:22.200 35.100010000 139.100020000 42.2000 1 0 0 0 0 0 0\\n",
    encoding="ascii",
)
""",
                encoding="utf-8",
            )
            fake_rtklib.chmod(0o755)

            args = argparse.Namespace(
                solver="rtk",
                rtklib_bin=fake_rtklib,
                rtklib_config=config_path,
                max_epochs=120,
            )

            elapsed = ppc_demo.run_rtklib_solver(
                args,
                rover,
                base,
                nav,
                ppc_demo.read_flexible_reference_csv(reference_csv),
                rtklib_pos,
            )

            self.assertGreaterEqual(elapsed, 0.0)
            self.assertTrue(rtklib_pos.exists())
            contents = rtklib_pos.read_text(encoding="ascii")
            self.assertIn("synthetic rtklib solution", contents)


class PPCMultiCandidateSelectorTest(unittest.TestCase):
    """Tests for apply_ppc_multi_candidate_selector."""

    @staticmethod
    def reference_epoch(index: int) -> comparison.ReferenceEpoch:
        return comparison.ReferenceEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([10.0 * index, 0.0, 0.0]),
        )

    @staticmethod
    def solution_epoch(
        index: int,
        ecef_x_m: float,
        status: int,
        post_rms_m: float | None,
    ) -> comparison.SolutionEpoch:
        return comparison.SolutionEpoch(
            2300,
            float(index),
            0.0,
            0.0,
            0.0,
            np.array([ecef_x_m, 0.0, 0.0]),
            status,
            12,
            10.0 if status == 4 else 0.0,
            100.0,
            2,
            16,
            8,
            8,
            0,
            post_rms_m,
            None if post_rms_m is None else post_rms_m * 4.0,
            post_rms_m,
            None if post_rms_m is None else post_rms_m * 4.0,
        )

    @staticmethod
    def write_reference_csv(path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(["tow", "week", "lat", "lon", "height", "ecef_x", "ecef_y", "ecef_z"])
            for index in range(3):
                writer.writerow([float(index), 2300, 0.0, 0.0, 0.0, 10.0 * index, 0.0, 0.0])

    def test_apply_ppc_multi_candidate_selector_emits_pos_and_summary(self) -> None:
        """Multi-candidate selector writes a .pos and summary JSON with expected keys."""
        reference = [self.reference_epoch(index) for index in range(3)]
        baseline = [
            self.solution_epoch(0, 0.0, 4, None),
            self.solution_epoch(1, 11.2, 3, None),
            self.solution_epoch(2, 20.1, 4, None),
        ]
        # nis5 candidate: segment 1 is FIXED (better than baseline FLOAT)
        nis5 = [
            self.solution_epoch(0, 0.0, 4, 0.2),
            self.solution_epoch(1, 10.1, 4, 0.3),
            self.solution_epoch(2, 21.0, 3, 0.2),
        ]
        # ratio4 candidate: segment 2 is FIXED (same baseline_m as nis5)
        ratio4 = [
            self.solution_epoch(0, 0.0, 4, 0.15),
            self.solution_epoch(1, 11.0, 3, 0.25),
            self.solution_epoch(2, 10.2, 4, 0.1),
        ]

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_multi_candidate_") as temp_dir:
            temp_root = Path(temp_dir)
            reference_csv = temp_root / "reference.csv"
            baseline_pos = temp_root / "baseline.pos"
            nis5_pos = temp_root / "nis5.pos"
            ratio4_pos = temp_root / "ratio4.pos"
            out_pos = temp_root / "selected.pos"
            summary_json = temp_root / "summary.json"
            segments_csv = temp_root / "segments.csv"

            self.write_reference_csv(reference_csv)
            ppc_dual_profile_selector.write_pos(baseline_pos, baseline)
            ppc_dual_profile_selector.write_pos(nis5_pos, nis5)
            ppc_dual_profile_selector.write_pos(ratio4_pos, ratio4)

            argv = [
                "apply_ppc_multi_candidate_selector.py",
                "--reference-csv", str(reference_csv),
                "--baseline-pos", str(baseline_pos),
                "--candidate", f"nis5={nis5_pos}",
                "--candidate", f"ratio4={ratio4_pos}",
                "--candidate-rule", "nis5=candidate_status_name == FIXED",
                "--candidate-rule", "ratio4=candidate_status_name == FIXED",
                "--priority-order", "nis5,ratio4",
                "--out-pos", str(out_pos),
                "--summary-json", str(summary_json),
                "--segments-csv", str(segments_csv),
            ]
            with mock.patch.object(sys, "argv", argv):
                ppc_multi_candidate_selector.main()

            self.assertTrue(out_pos.exists(), "selected .pos file must exist")
            self.assertTrue(summary_json.exists(), "summary JSON must exist")
            self.assertTrue(segments_csv.exists(), "segments CSV must exist")

            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertIn("metrics", payload)
            self.assertIn("baseline", payload)
            self.assertIn("delta_vs_baseline", payload)
            self.assertIn("selection", payload)
            self.assertIn("active_candidates", payload)
            self.assertIn("ppc_official_score_pct", payload["metrics"])

            reparsed = comparison.read_libgnss_pos(out_pos)
            self.assertGreater(len(reparsed), 0)

            metrics = ppc_metrics.summarize_solution_epochs(
                reference,
                reparsed,
                fixed_status=4,
                label="selected",
                match_tolerance_s=0.25,
                solver_wall_time_s=None,
            )
            self.assertGreaterEqual(
                metrics["ppc_official_score_pct"],
                payload["baseline"]["ppc_official_score_pct"],
            )

    def test_multi_candidate_selector_priority_first_ignores_reference_delta(self) -> None:
        """Deployable mode uses rule priority, not reference-scored best delta."""
        reference = [self.reference_epoch(index) for index in range(2)]
        baseline = [
            self.solution_epoch(0, 0.0, 4, None),
            self.solution_epoch(1, 10.0, 4, None),
        ]
        high_priority = [
            self.solution_epoch(0, 0.0, 4, 0.2),
            self.solution_epoch(1, 10.4, 4, 0.2),
        ]
        low_priority = [
            self.solution_epoch(0, 0.0, 4, 0.1),
            self.solution_epoch(1, 10.0, 4, 0.1),
        ]
        rule = ppc_multi_candidate_selector.parse_rule("candidate_all")
        candidate_specs = [
            ("high_priority", high_priority, rule),
            ("low_priority", low_priority, rule),
        ]
        baseline_records = [
            {
                "reference_index": 0,
                "solution_tow_s": 0.0,
                "start_tow_s": 0.0,
                "end_tow_s": 1.0,
                "segment_distance_m": 10.0,
            }
        ]
        candidate_rows = {
            "high_priority": [
                {
                    "reference_index": 0,
                    "score_delta_distance_m": -5.0,
                    "candidate_solution_tow_s": 0.0,
                    "candidate_status_name": "FIXED",
                }
            ],
            "low_priority": [
                {
                    "reference_index": 0,
                    "score_delta_distance_m": 5.0,
                    "candidate_solution_tow_s": 0.0,
                    "candidate_status_name": "FIXED",
                }
            ],
        }

        _, oracle_rows = ppc_multi_candidate_selector.select_segments(
            reference,
            baseline,
            candidate_specs,
            candidate_rows,
            ["high_priority", "low_priority"],
            0.25,
            baseline_records,
            {},
            selection_mode="oracle_delta",
        )
        _, priority_rows = ppc_multi_candidate_selector.select_segments(
            reference,
            baseline,
            candidate_specs,
            candidate_rows,
            ["high_priority", "low_priority"],
            0.25,
            baseline_records,
            {},
            selection_mode="priority_first",
        )

        self.assertEqual(oracle_rows[0]["selected_candidate"], "low_priority")
        self.assertEqual(priority_rows[0]["selected_candidate"], "high_priority")

    def test_multi_candidate_selector_drops_negative_candidate(self) -> None:
        """drop_negative_candidates removes any candidate whose selected net < 0.

        The function is called after select_segments.  We exercise it directly
        with a hand-crafted result_rows list where bad_cand has been selected on
        two segments with a net negative score delta, then verify that the
        returned active_specs no longer contains bad_cand.
        """
        good_epochs = [
            self.solution_epoch(0, 0.0, 4, 0.1),
            self.solution_epoch(1, 10.0, 4, 0.1),
            self.solution_epoch(2, 20.0, 4, 0.1),
        ]
        bad_epochs = [
            self.solution_epoch(0, 0.0, 4, 0.2),
            self.solution_epoch(1, 10.0, 4, 0.2),
            self.solution_epoch(2, 20.0, 4, 0.2),
        ]

        good_rule = ppc_multi_candidate_selector.parse_rule("candidate_all")
        bad_rule = ppc_multi_candidate_selector.parse_rule("candidate_all")

        candidate_specs = [
            ("good_cand", good_epochs, good_rule),
            ("bad_cand", bad_epochs, bad_rule),
        ]

        # Simulate result_rows where bad_cand was selected on two segments with
        # a net negative contribution (gained +5 on seg 0, lost -10 on seg 1).
        result_rows: list[dict[str, object]] = [
            {
                "reference_index": 0,
                "selected_candidate": "bad_cand",
                "score_delta_distance_m": 5.0,
            },
            {
                "reference_index": 1,
                "selected_candidate": "bad_cand",
                "score_delta_distance_m": -10.0,
            },
            {
                "reference_index": 2,
                "selected_candidate": "good_cand",
                "score_delta_distance_m": 3.0,
            },
        ]

        # bad_cand net = 5 + (-10) = -5 < 0 → must be dropped
        updated_specs, any_dropped = ppc_multi_candidate_selector.drop_negative_candidates(
            candidate_specs, {}, result_rows
        )

        self.assertTrue(any_dropped, "drop_negative_candidates must report a drop")
        active_labels = [label for label, _, _ in updated_specs]
        self.assertNotIn("bad_cand", active_labels, "bad_cand must be removed from active specs")
        self.assertIn("good_cand", active_labels, "good_cand must remain in active specs")


class PPCMultiCandidateSelectorMatrixTest(unittest.TestCase):
    """Tests for run_ppc_multi_candidate_selector_matrix."""

    # ------------------------------------------------------------------
    # Helpers shared by both tests
    # ------------------------------------------------------------------

    @staticmethod
    def _make_run_summary(
        bl_score_m: float,
        sel_score_m: float,
        total_m: float,
        candidate_selected_segments: int,
        active_candidates: list[str],
        dropped_candidates: list[str],
    ) -> dict[str, object]:
        """Build a minimal per-run summary JSON matching PR A's output shape."""
        bl_pct = 100.0 * bl_score_m / total_m if total_m > 0 else 0.0
        sel_pct = 100.0 * sel_score_m / total_m if total_m > 0 else 0.0
        delta_m = sel_score_m - bl_score_m
        delta_pct = sel_pct - bl_pct
        per_candidate: dict[str, object] = {}
        for label in active_candidates:
            per_candidate[label] = {
                "selected_segments": candidate_selected_segments,
                "score_delta_distance_m": delta_m / max(len(active_candidates), 1),
            }
        return {
            "active_candidates": active_candidates,
            "dropped_candidates": dropped_candidates,
            "priority_order": active_candidates,
            "baseline": {
                "ppc_official_score_pct": bl_pct,
                "ppc_official_score_distance_m": bl_score_m,
                "ppc_official_total_distance_m": total_m,
                "positioning_rate_pct": 80.0,
                "fix_rate_pct": 50.0,
            },
            "metrics": {
                "ppc_official_score_pct": sel_pct,
                "ppc_official_score_distance_m": sel_score_m,
                "ppc_official_total_distance_m": total_m,
                "positioning_rate_pct": 82.0,
                "fix_rate_pct": 52.0,
            },
            "delta_vs_baseline": {
                "ppc_official_score_pct": delta_pct,
                "ppc_official_score_distance_m": delta_m,
                "positioning_rate_pct": 2.0,
                "fix_rate_pct": 2.0,
            },
            "selection": {
                "segments": 3,
                "candidate_selected_segments": candidate_selected_segments,
                "baseline_selected_segments": 3 - candidate_selected_segments,
                "total_score_delta_distance_m": delta_m,
                "per_candidate": per_candidate,
            },
        }

    # ------------------------------------------------------------------
    # Test 1: subprocess invocation shape
    # ------------------------------------------------------------------

    def test_run_ppc_multi_candidate_selector_matrix_invokes_apply_per_run(
        self,
    ) -> None:
        """Matrix driver invokes apply_ppc_multi_candidate_selector.py once per run."""
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_multi_matrix_sub_") as td:
            temp_root = Path(td)
            output_json = temp_root / "matrix.json"
            # Pre-create a dummy summary JSON that invoke_apply_for_run would
            # normally produce so load_run_summary succeeds.
            dummy_summary = self._make_run_summary(
                100.0, 120.0, 200.0, 2, ["nis5"], []
            )

            captured_calls: list[list[str]] = []

            def fake_run(argv: list[str], check: bool) -> object:  # type: ignore[misc]
                captured_calls.append(argv)
                # Write the dummy summary so the driver can load it.
                # The argv has --summary-json at position -5 (or we search for it).
                try:
                    idx = argv.index("--summary-json")
                    summary_path = Path(argv[idx + 1])
                    summary_path.parent.mkdir(parents=True, exist_ok=True)
                    summary_path.write_text(
                        json.dumps(dummy_summary) + "\n", encoding="utf-8"
                    )
                except (ValueError, IndexError):
                    pass

                class _Result:
                    returncode = 0

                return _Result()

            dataset_root = temp_root / "PPC-Dataset"
            baseline_dir = temp_root / "baseline"
            baseline_dir.mkdir()

            argv = [
                "run_ppc_multi_candidate_selector_matrix.py",
                "--run",
                "tokyo/run1",
                "--run",
                "nagoya/run1",
                "--dataset-root",
                str(dataset_root),
                "--baseline-pos-template",
                str(baseline_dir / "{key}.pos"),
                "--candidate",
                "nis5=output/nis5/{key}.pos",
                "--candidate-rule",
                "nis5=candidate_status_name == FIXED",
                "--priority-order",
                "nis5",
                "--selection-mode",
                "priority_first",
                "--run-output-template",
                str(temp_root / "selected" / "{key}.pos"),
                "--summary-json",
                str(output_json),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch(
                    "run_ppc_multi_candidate_selector_matrix.subprocess.run",
                    side_effect=fake_run,
                ):
                    exit_code = ppc_multi_selector_matrix.main()

            self.assertEqual(exit_code, 0)
            # Two runs → two subprocess calls
            self.assertEqual(len(captured_calls), 2)
            run_keys_seen = set()
            for call_argv in captured_calls:
                # Must invoke apply_ppc_multi_candidate_selector.py
                self.assertIn(
                    "apply_ppc_multi_candidate_selector.py",
                    call_argv[1],
                    "Each subprocess call must target apply_ppc_multi_candidate_selector.py",
                )
                # Must pass --candidate and --candidate-rule
                self.assertIn("--candidate", call_argv)
                self.assertIn("--candidate-rule", call_argv)
                # Must pass --out-pos and --summary-json
                self.assertIn("--out-pos", call_argv)
                self.assertIn("--summary-json", call_argv)
                mode_idx = call_argv.index("--selection-mode")
                self.assertEqual(call_argv[mode_idx + 1], "priority_first")
                # Collect which run keys appeared in --baseline-pos
                try:
                    bl_idx = call_argv.index("--baseline-pos")
                    bl_path = call_argv[bl_idx + 1]
                    if "tokyo_run1" in bl_path:
                        run_keys_seen.add("tokyo_run1")
                    elif "nagoya_run1" in bl_path:
                        run_keys_seen.add("nagoya_run1")
                except (ValueError, IndexError):
                    pass
            self.assertIn("tokyo_run1", run_keys_seen)
            self.assertIn("nagoya_run1", run_keys_seen)

    # ------------------------------------------------------------------
    # Test 2: aggregation correctness
    # ------------------------------------------------------------------

    def test_run_ppc_multi_candidate_selector_matrix_aggregates_summary(
        self,
    ) -> None:
        """Aggregation over synthetic per-run payloads produces correct JSON+MD."""
        # tokyo_run1: baseline 100 m, selector 120 m, total 200 m
        # nagoya_run1: baseline 300 m, selector 330 m, total 600 m
        run_payloads: list[tuple[str, str, dict[str, object]]] = [
            (
                "tokyo",
                "run1",
                self._make_run_summary(100.0, 120.0, 200.0, 2, ["nis5"], []),
            ),
            (
                "nagoya",
                "run1",
                self._make_run_summary(300.0, 330.0, 600.0, 5, ["nis5"], ["bad"]),
            ),
        ]

        payload = ppc_multi_selector_matrix.build_payload(
            run_payloads, "Multi-candidate test"
        )
        aggregates = payload["aggregates"]

        # Weighted: (120+330)/(200+600)*100 = 56.25%
        self.assertAlmostEqual(
            aggregates["weighted_selector_official_score_pct"], 56.25, places=4
        )
        # Baseline weighted: (100+300)/(200+600)*100 = 50%
        self.assertAlmostEqual(
            aggregates["weighted_baseline_official_score_pct"], 50.0, places=4
        )
        # Delta: (120+330)-(100+300) = 50 m
        self.assertAlmostEqual(
            aggregates["selector_official_score_delta_m"], 50.0, places=4
        )
        # Total candidate-selected segments: 2+5=7
        self.assertEqual(aggregates["total_candidate_selected_segments"], 7)
        # dropped_candidates_any_run must include "bad"
        self.assertIn("bad", aggregates["dropped_candidates_any_run"])

        # Runs list
        runs = payload["runs"]
        self.assertEqual(len(runs), 2)
        self.assertEqual(runs[0]["key"], "tokyo_run1")
        self.assertEqual(runs[1]["key"], "nagoya_run1")

        # Markdown
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_multi_matrix_agg_") as td:
            summary_json = Path(td) / "matrix.json"
            markdown_path = Path(td) / "matrix.md"

            argv = [
                "run_ppc_multi_candidate_selector_matrix.py",
                "--summary-json",
                str(summary_json),
                "--markdown-output",
                str(markdown_path),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch(
                    "run_ppc_multi_candidate_selector_matrix.subprocess.run"
                ):
                    # We test render_markdown directly, not via main()
                    pass

            md = ppc_multi_selector_matrix.render_markdown(payload)
            self.assertIn("Multi-candidate test", md)
            self.assertIn("tokyo_run1", md)
            self.assertIn("nagoya_run1", md)
            self.assertIn("56.25", md)

            # build_payload output is deterministically sorted
            json_text = json.dumps(payload, indent=2, sort_keys=True)
            reparsed = json.loads(json_text)
            self.assertEqual(
                reparsed["aggregates"]["total_candidate_selected_segments"], 7
            )


class PPCRatioGatingSelectorSweepTest(unittest.TestCase):
    """Tests for run_ppc_ratio_gating_selector_sweep."""

    @staticmethod
    def _matrix_payload(
        baseline_pct: float,
        selector_pct: float,
        delta_m: float,
        candidate_segments: int,
    ) -> dict[str, object]:
        return {
            "title": "synthetic matrix",
            "candidates": ["jump", "olddef"],
            "aggregates": {
                "run_count": 2,
                "official_total_distance_m": 1000.0,
                "weighted_baseline_official_score_pct": baseline_pct,
                "weighted_selector_official_score_pct": selector_pct,
                "selector_official_score_delta_pct": selector_pct - baseline_pct,
                "selector_official_score_delta_m": delta_m,
                "min_official_score_delta_m": delta_m / 4.0,
                "max_official_score_delta_m": delta_m / 2.0,
                "total_candidate_selected_segments": candidate_segments,
            },
            "runs": [],
        }

    def test_threshold_set_parser_supports_common_wide_and_per_candidate(self) -> None:
        labels = ["jump", "olddef"]

        wide_name, wide = ppc_ratio_gating_sweep.parse_threshold_set("wide=none", labels)
        self.assertEqual(wide_name, "wide")
        self.assertIsNone(wide["jump"])
        self.assertEqual(
            ppc_ratio_gating_sweep.threshold_rule("jump", wide["jump"]),
            "jump=candidate_status_name == FIXED",
        )

        tight_name, tight = ppc_ratio_gating_sweep.parse_threshold_set(
            "tight:jump=4,olddef=5", labels
        )
        self.assertEqual(tight_name, "tight")
        self.assertEqual(tight["jump"], 4.0)
        self.assertEqual(tight["olddef"], 5.0)
        self.assertEqual(
            ppc_ratio_gating_sweep.threshold_rule("olddef", tight["olddef"]),
            "olddef=candidate_status_name == FIXED AND candidate_ratio >= 5",
        )

    def test_ratio_gating_sweep_invokes_matrix_and_writes_pareto_outputs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_ratio_gating_") as td:
            temp_root = Path(td)
            output_dir = temp_root / "sweep"
            summary_json = temp_root / "summary.json"
            markdown_output = temp_root / "summary.md"

            captured_calls: list[list[str]] = []

            def fake_run(argv: list[str], check: bool) -> object:  # type: ignore[misc]
                captured_calls.append(argv)
                summary_idx = argv.index("--summary-json")
                matrix_json = Path(argv[summary_idx + 1])
                matrix_json.parent.mkdir(parents=True, exist_ok=True)
                payload = (
                    self._matrix_payload(21.0, 40.0, 900.0, 80)
                    if "wide" in matrix_json.parts
                    else self._matrix_payload(21.0, 35.0, 650.0, 50)
                )
                matrix_json.write_text(json.dumps(payload) + "\n", encoding="utf-8")

                markdown_idx = argv.index("--markdown-output")
                matrix_md = Path(argv[markdown_idx + 1])
                matrix_md.write_text("# matrix\n", encoding="utf-8")

                class _Result:
                    returncode = 0

                return _Result()

            argv = [
                "run_ppc_ratio_gating_selector_sweep.py",
                "--run",
                "tokyo/run1",
                "--dataset-root",
                str(temp_root / "PPC-Dataset"),
                "--baseline-pos-template",
                str(temp_root / "baseline" / "{key}.pos"),
                "--candidate",
                "jump=output/jump/{key}.pos",
                "--candidate",
                "olddef=output/olddef/{key}.pos",
                "--priority-order",
                "jump,olddef",
                "--selection-mode",
                "priority_first",
                "--threshold-set",
                "wide=none",
                "--threshold-set",
                "tight:jump=4,olddef=5",
                "--output-dir",
                str(output_dir),
                "--summary-json",
                str(summary_json),
                "--markdown-output",
                str(markdown_output),
            ]

            with mock.patch.object(sys, "argv", argv):
                with mock.patch(
                    "run_ppc_ratio_gating_selector_sweep.subprocess.run",
                    side_effect=fake_run,
                ):
                    exit_code = ppc_ratio_gating_sweep.main()

            self.assertEqual(exit_code, 0)
            self.assertEqual(len(captured_calls), 2)
            self.assertTrue(summary_json.exists())
            self.assertTrue(markdown_output.exists())

            wide_call = next(call for call in captured_calls if "wide" in call[call.index("--summary-json") + 1])
            tight_call = next(call for call in captured_calls if "tight" in call[call.index("--summary-json") + 1])
            wide_rules = [
                wide_call[idx + 1]
                for idx, value in enumerate(wide_call)
                if value == "--candidate-rule"
            ]
            tight_rules = [
                tight_call[idx + 1]
                for idx, value in enumerate(tight_call)
                if value == "--candidate-rule"
            ]
            self.assertIn("jump=candidate_status_name == FIXED", wide_rules)
            self.assertIn(
                "jump=candidate_status_name == FIXED AND candidate_ratio >= 4",
                tight_rules,
            )
            self.assertIn(
                "olddef=candidate_status_name == FIXED AND candidate_ratio >= 5",
                tight_rules,
            )
            mode_idx = wide_call.index("--selection-mode")
            self.assertEqual(wide_call[mode_idx + 1], "priority_first")

            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["sets"][0]["name"], "wide")
            self.assertEqual(payload["sets"][1]["name"], "tight")
            self.assertEqual(payload["sets"][0]["selector_official_score_delta_pct"], 19.0)
            md_text = markdown_output.read_text(encoding="utf-8")
            self.assertIn("wide", md_text)
            self.assertIn("tight", md_text)
            self.assertIn("ratio>=4", md_text)


class PPCMultiCandidateSelectorAnalyzerTest(unittest.TestCase):
    """Tests for analyze_ppc_multi_candidate_selector_matrix."""

    # ------------------------------------------------------------------
    # Synthetic matrix JSON fixture
    # ------------------------------------------------------------------

    @staticmethod
    def _make_matrix_payload() -> dict[str, object]:
        """Build a minimal matrix-level JSON matching PR B's output schema."""

        def run_entry(
            city: str,
            run_name: str,
            bl_score_m: float,
            sel_score_m: float,
            total_m: float,
            cand_segs: int,
        ) -> dict[str, object]:
            key = f"{city}_{run_name}"
            bl_pct = 100.0 * bl_score_m / total_m
            sel_pct = 100.0 * sel_score_m / total_m
            delta_m = sel_score_m - bl_score_m
            delta_pct = sel_pct - bl_pct
            return {
                "key": key,
                "city": city,
                "run": run_name,
                "active_candidates": ["nis5", "jump0p5"],
                "dropped_candidates": [],
                "priority_order": ["nis5", "jump0p5"],
                "baseline": {
                    "ppc_official_score_pct": bl_pct,
                    "ppc_official_score_distance_m": bl_score_m,
                    "ppc_official_total_distance_m": total_m,
                },
                "selector": {
                    "ppc_official_score_pct": sel_pct,
                    "ppc_official_score_distance_m": sel_score_m,
                    "ppc_official_total_distance_m": total_m,
                },
                "delta_vs_baseline": {
                    "ppc_official_score_pct": delta_pct,
                    "ppc_official_score_distance_m": delta_m,
                },
                "selection": {
                    "segments": 5,
                    "candidate_selected_segments": cand_segs,
                    "baseline_selected_segments": 5 - cand_segs,
                    "total_score_delta_distance_m": delta_m,
                },
                "per_candidate": {
                    "nis5": {
                        "selected_segments": cand_segs,
                        "score_delta_distance_m": delta_m * 0.6,
                    },
                    "jump0p5": {
                        "selected_segments": 0,
                        "score_delta_distance_m": 0.0,
                    },
                },
            }

        runs = [
            run_entry("tokyo", "run1", 100.0, 120.0, 200.0, 2),
            run_entry("nagoya", "run1", 300.0, 330.0, 600.0, 4),
        ]
        total_m = 800.0
        bl_total = 400.0
        sel_total = 450.0
        return {
            "title": "PPC multi-candidate test",
            "candidates": ["nis5", "jump0p5"],
            "aggregates": {
                "run_count": 2,
                "official_total_distance_m": total_m,
                "weighted_baseline_official_score_pct": 100.0 * bl_total / total_m,
                "weighted_selector_official_score_pct": 100.0 * sel_total / total_m,
                "selector_official_score_delta_m": sel_total - bl_total,
                "selector_official_score_delta_pct": (
                    100.0 * sel_total / total_m - 100.0 * bl_total / total_m
                ),
                "min_official_score_delta_m": 20.0,
                "max_official_score_delta_m": 30.0,
                "total_candidate_selected_segments": 6,
                "total_baseline_selected_segments": 4,
                "total_score_delta_distance_m": 50.0,
                "dropped_candidates_any_run": [],
            },
            "runs": runs,
        }

    # ------------------------------------------------------------------
    # Test 1: markdown output
    # ------------------------------------------------------------------

    def test_analyze_ppc_multi_candidate_selector_matrix_emits_markdown(
        self,
    ) -> None:
        """Analyzer emits markdown with per-run rows and aggregate totals."""
        payload = self._make_matrix_payload()

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_multi_analyzer_md_") as td:
            temp_root = Path(td)
            summary_json = temp_root / "matrix.json"
            markdown_out = temp_root / "matrix.md"

            summary_json.write_text(json.dumps(payload), encoding="utf-8")

            argv = [
                "analyze_ppc_multi_candidate_selector_matrix.py",
                "--summary-json",
                str(summary_json),
                "--markdown-output",
                str(markdown_out),
            ]
            with mock.patch.object(sys, "argv", argv):
                exit_code = ppc_multi_cand_analyzer.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(markdown_out.exists())
            md_text = markdown_out.read_text(encoding="utf-8")

            # Title present
            self.assertIn("PPC multi-candidate test", md_text)
            # Per-run rows present — render_markdown uses run_label() so keys
            # appear as "Nagoya r1" / "Tokyo r1" in the rows.
            self.assertIn("Nagoya r1", md_text)
            self.assertIn("Tokyo r1", md_text)
            # Aggregate score values present
            self.assertIn("56.25", md_text)  # 450/800*100
            self.assertIn("50.00", md_text)  # 400/800*100
            # Section headers
            self.assertIn("## Aggregate", md_text)
            self.assertIn("## Runs", md_text)

    # ------------------------------------------------------------------
    # Test 2: PNG output
    # ------------------------------------------------------------------

    def test_analyze_ppc_multi_candidate_selector_matrix_writes_png(
        self,
    ) -> None:
        """Analyzer writes a non-trivial PNG scorecard when --scorecard is given."""
        payload = self._make_matrix_payload()

        with tempfile.TemporaryDirectory(prefix="gnss_ppc_multi_analyzer_png_") as td:
            temp_root = Path(td)
            summary_json = temp_root / "matrix.json"
            scorecard_png = temp_root / "scorecard.png"

            summary_json.write_text(json.dumps(payload), encoding="utf-8")

            argv = [
                "analyze_ppc_multi_candidate_selector_matrix.py",
                "--summary-json",
                str(summary_json),
                "--scorecard",
                str(scorecard_png),
            ]
            with mock.patch.object(sys, "argv", argv):
                with mock.patch.dict(os.environ, {"MPLBACKEND": "Agg"}, clear=False):
                    exit_code = ppc_multi_cand_analyzer.main()

            self.assertEqual(exit_code, 0)
            self.assertTrue(scorecard_png.exists(), "Scorecard PNG must be created")
            self.assertGreater(
                scorecard_png.stat().st_size,
                1024,
                "Scorecard PNG must be larger than 1 KB",
            )


if __name__ == "__main__":
    unittest.main()
