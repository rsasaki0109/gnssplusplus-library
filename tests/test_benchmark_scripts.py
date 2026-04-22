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
import generate_ppc_rtk_scorecard as ppc_rtk_scorecard  # noqa: E402
import generate_ppc_tail_cleanup_scorecard as ppc_tail_cleanup_scorecard  # noqa: E402
import generate_ppc_rtk_trajectory as ppc_rtk_trajectory  # noqa: E402
import update_ppc_coverage_readme as ppc_coverage_readme  # noqa: E402
import detect_ci_scope as ci_scope  # noqa: E402
import run_optional_ppp_products_signoff as ci_ppp_products_signoff  # noqa: E402
import run_optional_rtk_signoffs as ci_rtk_signoffs  # noqa: E402


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
                "max_consec_float_reset": 10,
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
            self.assertIn("--max-consec-float-reset", command)
            self.assertIn("10", command)
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
                max_consec_float_reset=10,
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
            self.assertIn("--max-consec-float-reset", command)
            self.assertIn("10", command)
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
            self.assertEqual(payload["aggregates"]["float_bridge_tail_rejected_epochs"], 147)
            self.assertEqual(payload["aggregates"]["fixed_bridge_burst_rejected_epochs"], 12)
            self.assertIsNone(payload["max_pos_jump_min"])
            self.assertIsNone(payload["max_pos_jump_rate"])
            self.assertIsNone(payload["max_consec_float_reset"])
            self.assertFalse(payload["enable_wide_lane_ar"])
            self.assertIsNone(payload["wide_lane_threshold"])
            self.assertIn("tokyo_run1", markdown)
            self.assertIn("+19.9 pp", markdown)
            self.assertIn("42.0%", markdown)
            self.assertIn("PPC official weighted delta: 21.0 pp", markdown)
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
                    ),
                )


class PPCMetricsTest(unittest.TestCase):
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
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([10.2, 0.0, 0.0]), 4, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([21.0, 0.0, 0.0]), 3, 12),
        ]
        rtklib_solution = [
            comparison.SolutionEpoch(2300, 1.0, 0.0, 0.0, 0.0, np.array([11.0, 0.0, 0.0]), 2, 12),
            comparison.SolutionEpoch(2300, 2.0, 0.0, 0.0, 0.0, np.array([20.1, 0.0, 0.0]), 1, 12),
        ]
        lib_records = ppc_metrics.ppc_official_segment_records(reference, lib_solution, 0.25)
        rtklib_records = ppc_metrics.ppc_official_segment_records(reference, rtklib_solution, 0.25)

        loss_by_state = {
            row["score_state"]: row
            for row in ppc_coverage_quality.official_loss_by_state(lib_records)
        }
        self.assertEqual(loss_by_state["scored"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["high_error"]["distance_m"], 10.0)
        self.assertEqual(loss_by_state["no_solution"]["distance_m"], 20.0)

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
            max_consec_float_reset=10,
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
        self.assertIn("--max-consec-float-reset", commands[0])
        self.assertIn("10", commands[0])
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
                max_consec_float_reset=10,
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
            self.assertEqual(payload["rtk_max_consecutive_float_for_reset"], 10)
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


if __name__ == "__main__":
    unittest.main()
