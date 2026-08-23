"""Benchmark sign-off and CI helper test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "PPCRTKSignoffHelpersTest",
    "PPCCommercialHelpersTest",
    "PublicRTKBenchmarksTest",
    "SmartLocAdapterTest",
    "SmartLocSignoffTest",
    "OptionalRTKSignoffScriptTest",
    "OptionalPPPProductsSignoffScriptTest",
    "CIScopeDetectionTest",
    "MovingBaseSignoffHelpersTest",
    "ShortBaselineSignoffTest",
    "PPPStaticSignoffTest",
    "PPPKinematicSignoffTest",
    "LiveSignoffTest",
]


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
                max_fixed_update_nis_per_obs=None,
                max_fixed_update_post_rms=None,
                max_fixed_update_gate_ratio=None,
                max_postfix_rms=None,
                enable_wide_lane_ar=False,
                wide_lane_threshold=None,
                enable_wlnl_fallback=False,
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
                "carrier_phase_sigma": 0.001,
                "max_subset_ar_drop_steps": 18,
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
                "max_fixed_update_nis_per_obs": 10.0,
                "max_fixed_update_post_rms": 6.0,
                "max_fixed_update_gate_ratio": 8.0,
                "min_fixed_update_gate_baseline": 7000.0,
                "max_fixed_update_gate_baseline": 8200.0,
                "min_fixed_update_gate_speed": 5.0,
                "max_fixed_update_gate_speed": 15.0,
                "max_fixed_update_secondary_gate_ratio": 4.0,
                "min_fixed_update_secondary_gate_baseline": 2000.0,
                "max_fixed_update_secondary_gate_baseline": 2500.0,
                "min_fixed_update_secondary_gate_speed": 7.0,
                "max_fixed_update_secondary_gate_speed": 15.0,
                "demote_fixed_status_nis_per_obs": 20.0,
                "demote_fixed_status_post_rms": 3.0,
                "demote_fixed_status_gate_ratio": 6.0,
                "demote_fixed_status_min_satellites": 9,
                "demote_fixed_status_low_satellite_ceiling": 11,
                "demote_fixed_status_low_satellite_max_ratio": 15.0,
                "max_fixed_prefit_rms": 12.0,
                "min_fixed_prefit_outliers": 45,
                "fixed_prefit_reset_streak": 1,
                "min_demote_fixed_status_baseline": 500.0,
                "max_demote_fixed_status_baseline": 9500.0,
                "rtk_snr_weighting": True,
                "rtk_snr_reference_dbhz": 44.0,
                "rtk_snr_max_variance_scale": 16.0,
                "rtk_snr_min_baseline": 7000.0,
                "cycle_slip_threshold": 0.08,
                "doppler_slip_threshold": 0.15,
                "code_slip_threshold": 4.0,
                "strict_dynamic_slip_thresholds": True,
                "adaptive_dynamic_slip_thresholds": True,
                "adaptive_dynamic_slip_nonfix_count": 2,
                "adaptive_dynamic_slip_hold_epochs": 8,
                "max_consec_float_reset": 10,
                "max_consec_nonfix_reset": 10,
                "max_postfix_rms": 0.20,
                "enable_wide_lane_ar": True,
                "wide_lane_threshold": 0.10,
                "enable_wlnl_fallback": True,
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
            self.assertIn("--carrier-phase-sigma", command)
            self.assertIn("0.001", command)
            self.assertIn("--max-subset-ar-drop-steps", command)
            self.assertIn("18", command)
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
            self.assertIn("--max-fixed-update-nis-per-obs", command)
            self.assertIn("10.0", command)
            self.assertIn("--max-fixed-update-post-rms", command)
            self.assertIn("6.0", command)
            self.assertIn("--max-fixed-update-gate-ratio", command)
            self.assertIn("8.0", command)
            self.assertIn("--min-fixed-update-gate-baseline", command)
            self.assertIn("7000.0", command)
            self.assertIn("--max-fixed-update-gate-baseline", command)
            self.assertIn("8200.0", command)
            self.assertIn("--demote-fixed-status-nis-per-obs", command)
            self.assertIn("20.0", command)
            self.assertIn("--demote-fixed-status-post-rms", command)
            self.assertIn("3.0", command)
            self.assertIn("--demote-fixed-status-gate-ratio", command)
            self.assertIn("6.0", command)
            self.assertIn("--demote-fixed-status-min-satellites", command)
            self.assertIn("9", command)
            self.assertIn("--demote-fixed-status-low-satellite-ceiling", command)
            self.assertIn("11", command)
            self.assertIn("--demote-fixed-status-low-satellite-max-ratio", command)
            self.assertIn("15.0", command)
            self.assertIn("--max-fixed-prefit-rms", command)
            self.assertIn("12.0", command)
            self.assertIn("--min-fixed-prefit-outliers", command)
            self.assertIn("45", command)
            self.assertIn("--fixed-prefit-reset-streak", command)
            self.assertIn("--min-demote-fixed-status-baseline", command)
            self.assertIn("500.0", command)
            self.assertIn("--max-demote-fixed-status-baseline", command)
            self.assertIn("9500.0", command)
            self.assertIn("--rtk-snr-weighting", command)
            self.assertIn("--rtk-snr-reference-dbhz", command)
            self.assertIn("44.0", command)
            self.assertIn("--rtk-snr-max-variance-scale", command)
            self.assertIn("16.0", command)
            self.assertIn("--rtk-snr-min-baseline", command)
            self.assertIn("7000.0", command)
            self.assertIn("--cycle-slip-threshold", command)
            self.assertIn("0.08", command)
            self.assertIn("--doppler-slip-threshold", command)
            self.assertIn("0.15", command)
            self.assertIn("--code-slip-threshold", command)
            self.assertIn("4.0", command)
            self.assertIn("--strict-dynamic-slip-thresholds", command)
            self.assertIn("--adaptive-dynamic-slip-thresholds", command)
            self.assertIn("--adaptive-dynamic-slip-nonfix-count", command)
            self.assertIn("2", command)
            self.assertIn("--adaptive-dynamic-slip-hold-epochs", command)
            self.assertIn("8", command)
            self.assertIn("--max-consec-float-reset", command)
            self.assertIn("10", command)
            self.assertIn("--max-consec-nonfix-reset", command)
            self.assertIn("--max-postfix-rms", command)
            self.assertIn("0.2", command)
            self.assertIn("--enable-wide-lane-ar", command)
            self.assertIn("--wide-lane-threshold", command)
            self.assertIn("0.1", command)
            self.assertIn("--enable-wlnl-fallback", command)
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
            realtime_profile="city-default",
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

        realtime_args = argparse.Namespace(
            realtime_profile="sigma-demote",
            preset=None,
            arfilter=None,
            arfilter_margin=None,
            min_hold_count=None,
            hold_ratio_threshold=None,
            no_kinematic_post_filter=False,
        )
        realtime = ppc_rtk_signoff.selected_tuning(realtime_args, "tokyo")
        self.assertEqual(realtime["preset"], "low-cost")
        self.assertEqual(realtime["ratio"], 2.8)
        self.assertEqual(realtime["carrier_phase_sigma"], 0.001)
        self.assertEqual(realtime["demote_fixed_status_nis_per_obs"], 2.0)
        self.assertNotIn("demote_fixed_status_gate_ratio", realtime)
        self.assertNotIn("max_demote_fixed_status_baseline", realtime)
        self.assertFalse(realtime["arfilter"])
        self.assertTrue(realtime["no_kinematic_post_filter"])

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
        adapter_contracts = public_rtk_benchmarks.select_profiles(["adapter-contract"])
        adapter_contract_ids = {profile.profile_id for profile in adapter_contracts}
        markdown = public_rtk_benchmarks.render_markdown(candidates)

        self.assertNotIn("gsdc", candidate_ids)
        self.assertIn("gsdc", adapter_contract_ids)
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
                [
                    "ppc_coverage_matrix_schema_smoke",
                    "ppc_nagoya_run1_rtk",
                    "ppc_tokyo_run1_rtk",
                    "ppc_taroz_amb_pdc_nagoya_run3_1000_seed",
                    "scorpion_moving_base",
                ],
            )
            self.assertTrue(all(step.command is None for step in steps))
            self.assertEqual(steps[0].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[1].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[2].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[3].skip_reason, "PPC-Dataset root is unavailable.")
            self.assertEqual(steps[4].skip_reason, "SCORPION moving-base input is unavailable.")

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

            coverage, nagoya, tokyo, taroz_long, scorpion = steps
            self.assertIsNotNone(coverage.command)
            self.assertIn("ppc-coverage-matrix", coverage.command)
            self.assertIn("--config-toml", coverage.command)
            self.assertIn("--max-epochs", coverage.command)
            self.assertIn("2", coverage.command)
            self.assertIn("--summary-json", coverage.command)
            self.assertIn(str(output_dir / "ppc_coverage_matrix_schema_smoke" / "summary.json"), coverage.command)
            self.assertIsNotNone(nagoya.command)
            self.assertIn("--dataset-root", nagoya.command)
            self.assertIn(str(dataset_root), nagoya.command)
            self.assertIsNotNone(tokyo.command)
            self.assertIn("--rtklib-bin", tokyo.command)
            self.assertIn(str(rtklib_bin), tokyo.command)
            self.assertIn("--rtklib-pos", tokyo.command)
            self.assertIsNotNone(taroz_long.command)
            self.assertIn("ppc-taroz-amb-pdc-smoke", taroz_long.command)
            self.assertIn("--dataset-root", taroz_long.command)
            self.assertIn(str(dataset_root), taroz_long.command)
            self.assertIn("--run", taroz_long.command)
            self.assertIn("nagoya/run3", taroz_long.command)
            self.assertIn("--max-epochs", taroz_long.command)
            self.assertIn("1000", taroz_long.command)
            self.assertIn("--generate-spp-seed", taroz_long.command)
            self.assertIn("--require-valid-p95-3d-max", taroz_long.command)
            self.assertIn("1.1", taroz_long.command)
            self.assertIn("--require-fixed-p95-3d-max", taroz_long.command)
            self.assertIn("0.2", taroz_long.command)
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

            coverage, nagoya, tokyo, taroz_long, _ = steps
            self.assertIsNotNone(coverage.command)
            self.assertIsNotNone(nagoya.command)
            self.assertIsNone(tokyo.command)
            self.assertEqual(tokyo.skip_reason, "RTKLIB binary is unavailable.")
            self.assertIsNotNone(taroz_long.command)

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
            self.assertEqual(passed_result["missing_outputs"], [])
            self.assertIn("ok", (log_dir / "pass_example.log").read_text(encoding="utf-8"))

            missing_output = ci_rtk_signoffs.SignoffStep(
                name="Missing output example",
                slug="missing_output_example",
                command=[sys.executable, "-c", "print('ok')"],
                outputs=[str(temp_root / "missing.json")],
            )
            missing_result = ci_rtk_signoffs.run_step(missing_output, ROOT_DIR, log_dir)
            self.assertEqual(missing_result["status"], "failed")
            self.assertEqual(missing_result["returncode"], 0)
            self.assertEqual(
                missing_result["missing_outputs"],
                [str(temp_root / "missing.json")],
            )

    def test_write_summary_records_schema_and_counts(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ci_rtk_summary_") as temp_dir:
            temp_root = Path(temp_dir)
            summary_path = temp_root / "summary.json"
            steps = [
                ci_rtk_signoffs.SignoffStep(
                    name="PPC coverage matrix schema smoke",
                    slug="ppc_coverage_matrix_schema_smoke",
                    command=None,
                    outputs=[],
                    skip_reason="PPC-Dataset root is unavailable.",
                )
            ]
            results = [
                {
                    "name": steps[0].name,
                    "slug": steps[0].slug,
                    "status": "skipped",
                    "skip_reason": steps[0].skip_reason,
                    "outputs": [],
                    "log_path": str(temp_root / "ppc_coverage_matrix_schema_smoke.log"),
                }
            ]

            ci_rtk_signoffs.write_summary(summary_path, steps, results)

            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["summary_schema"], ci_rtk_signoffs.SUMMARY_SCHEMA)
            self.assertEqual(payload["counts"], {"passed": 0, "failed": 0, "skipped": 1})
            self.assertEqual(payload["steps"][0]["slug"], "ppc_coverage_matrix_schema_smoke")

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
                {
                    "name": "PPC coverage matrix schema smoke",
                    "status": "failed",
                    "missing_outputs": ["/tmp/summary.json"],
                    "log_path": "/tmp/d.log",
                },
            ]
        )

        self.assertIn("## Optional RTK Sign-offs", markdown)
        self.assertIn("`passed`: `1`", markdown)
        self.assertIn("`failed`: `2`", markdown)
        self.assertIn("`skipped`: `1`", markdown)
        self.assertIn("| PPC Nagoya RTK sign-off | `passed` | 1.25s |", markdown)
        self.assertIn("RTKLIB binary is unavailable.", markdown)
        self.assertIn("see `/tmp/c.log`", markdown)
        self.assertIn("missing `/tmp/summary.json`", markdown)

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
                "docs/archive/2026-04-11_ci.md",
                "README.md",
                "scripts/generate_architecture_diagram.py",
            ]
        )

        self.assertEqual(
            payload["changed_paths"],
            [
                "README.md",
                "docs/archive/2026-04-11_ci.md",
                "docs/guide.md",
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
