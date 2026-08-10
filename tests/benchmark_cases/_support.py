"""Shared imports and repository path bootstrap for benchmark test modules."""

from __future__ import annotations

import argparse
import dataclasses
import csv
from datetime import timedelta
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np


ROOT_DIR = Path(__file__).resolve().parents[2]
COMMANDS_DIR = ROOT_DIR / "apps" / "commands"
SCRIPTS_DIR = ROOT_DIR / "scripts"
ANALYSIS_DIR = SCRIPTS_DIR / "analysis"
CI_SCRIPTS_DIR = SCRIPTS_DIR / "ci"
PPC_EXPERIMENTS_DIR = SCRIPTS_DIR / "experiments" / "ppc"

for command_group in (
    "benchmarks", "positioning", "products", "receivers",
    "visualization", "diagnostics",
):
    sys.path.insert(0, str(COMMANDS_DIR / command_group))
sys.path.insert(0, str(COMMANDS_DIR))
sys.path.insert(0, str(SCRIPTS_DIR))
sys.path.insert(0, str(ANALYSIS_DIR))
sys.path.insert(0, str(CI_SCRIPTS_DIR))
sys.path.insert(0, str(PPC_EXPERIMENTS_DIR))

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
import gnss_ppp_iers_atm_tidal_loading_multisite_bench as iers_atl_multisite_bench  # noqa: E402
import gnss_ppp_iers_pole_tide_multisite_bench as iers_multisite_bench  # noqa: E402
import gnss_vmf_atl as vmf_atl  # noqa: E402
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
import apply_ppc_integrity_consensus as ppc_integrity_consensus  # noqa: E402
import generate_ppc_rtk_scorecard as ppc_rtk_scorecard  # noqa: E402
import generate_ppc_goal_scorecard as ppc_goal_scorecard  # noqa: E402
import plot_ppc_status_trajectories as ppc_status_trajectories  # noqa: E402
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
import run_clas_mrtklib_v051 as clas_v051_runner  # noqa: E402
import fuse_kf_fgo_alignment as kf_fgo_alignment  # noqa: E402
import evaluate_ppc_fgo_shadow_authority as fgo_shadow_authority  # noqa: E402
import apply_ppc_fgo_position_consensus as fgo_position_consensus  # noqa: E402
import summarize_fgo_ppc_matrix as fgo_ppc_matrix  # noqa: E402
import bridge_pos_fixed_anchors as fixed_anchor_bridge  # noqa: E402
import smooth_pos_float_horizontal as float_horizontal_smoother  # noqa: E402
import bridge_pos_velocity_fixed_anchors as velocity_fixed_anchor_bridge  # noqa: E402
import select_pos_candidate_quality as candidate_quality_selector  # noqa: E402
import convert_tcfgo_npz_to_pos as tcfgo_converter  # noqa: E402
import merge_pos_status_transition as status_position_merge  # noqa: E402

# Keep the original script imports centralized so domain modules share one bootstrap.
