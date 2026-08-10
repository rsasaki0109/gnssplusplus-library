"""PPC multi-candidate selector benchmark test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "PPCDemoTest",
    "PPCMultiCandidateSelectorTest",
    "PPCMultiCandidateSelectorMatrixTest",
    "PPCRatioGatingSelectorSweepTest",
    "PPCMultiCandidateSelectorAnalyzerTest",
]


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
            max_subset_ar_drop_steps=18,
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
            max_fixed_update_nis_per_obs=10.0,
            max_fixed_update_post_rms=6.0,
            max_fixed_update_gate_ratio=8.0,
            min_fixed_update_gate_baseline=7000.0,
            max_fixed_update_gate_baseline=8200.0,
            min_fixed_update_gate_speed=5.0,
            max_fixed_update_gate_speed=15.0,
            max_fixed_update_secondary_gate_ratio=4.0,
            min_fixed_update_secondary_gate_baseline=2000.0,
            max_fixed_update_secondary_gate_baseline=2500.0,
            min_fixed_update_secondary_gate_speed=7.0,
            max_fixed_update_secondary_gate_speed=15.0,
            max_consec_float_reset=10,
            max_consec_nonfix_reset=10,
            max_postfix_rms=0.20,
            enable_wide_lane_ar=True,
            wide_lane_threshold=0.10,
            enable_wlnl_fallback=True,
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
        self.assertIn("--max-subset-ar-drop-steps", commands[0])
        self.assertIn("18", commands[0])
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
        self.assertIn("--max-fixed-update-nis-per-obs", commands[0])
        self.assertIn("10.0", commands[0])
        self.assertIn("--max-fixed-update-post-rms", commands[0])
        self.assertIn("6.0", commands[0])
        self.assertIn("--max-fixed-update-gate-ratio", commands[0])
        self.assertIn("8.0", commands[0])
        self.assertIn("--min-fixed-update-gate-baseline", commands[0])
        self.assertIn("7000.0", commands[0])
        self.assertIn("--max-fixed-update-gate-baseline", commands[0])
        self.assertIn("8200.0", commands[0])
        self.assertIn("--min-fixed-update-gate-speed", commands[0])
        self.assertIn("5.0", commands[0])
        self.assertIn("--max-fixed-update-gate-speed", commands[0])
        self.assertIn("15.0", commands[0])
        self.assertIn("--max-fixed-update-secondary-gate-ratio", commands[0])
        self.assertIn("4.0", commands[0])
        self.assertIn("--min-fixed-update-secondary-gate-baseline", commands[0])
        self.assertIn("2000.0", commands[0])
        self.assertIn("--max-fixed-update-secondary-gate-baseline", commands[0])
        self.assertIn("2500.0", commands[0])
        self.assertIn("--min-fixed-update-secondary-gate-speed", commands[0])
        self.assertIn("7.0", commands[0])
        self.assertIn("--max-fixed-update-secondary-gate-speed", commands[0])
        self.assertIn("15.0", commands[0])
        self.assertIn("--max-consec-float-reset", commands[0])
        self.assertIn("10", commands[0])
        self.assertIn("--max-consec-nonfix-reset", commands[0])
        self.assertIn("--max-postfix-rms", commands[0])
        self.assertIn("0.2", commands[0])
        self.assertIn("--enable-wide-lane-ar", commands[0])
        self.assertIn("--wide-lane-threshold", commands[0])
        self.assertIn("0.1", commands[0])
        self.assertIn("--enable-wlnl-fallback", commands[0])
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
                max_subset_ar_drop_steps=18,
                max_hold_div=5.0,
                max_pos_jump=20.0,
                max_fixed_anchor_age=30.0,
                max_fixed_doppler_consensus=10.0,
                max_pos_jump_min=20.0,
                max_pos_jump_rate=25.0,
                max_float_spp_div=30.0,
                max_float_prefit_rms=6.0,
                max_float_prefit_max=30.0,
                max_float_prefit_reset_streak=5,
                min_float_prefit_trusted_jump=8.0,
                max_update_nis_per_obs=12.0,
                max_fixed_update_nis_per_obs=10.0,
                max_fixed_update_post_rms=6.0,
                max_fixed_update_gate_ratio=8.0,
                min_fixed_update_gate_baseline=7000.0,
                max_fixed_update_gate_baseline=8200.0,
                min_fixed_update_gate_speed=5.0,
                max_fixed_update_gate_speed=15.0,
                max_fixed_update_secondary_gate_ratio=4.0,
                min_fixed_update_secondary_gate_baseline=2000.0,
                max_fixed_update_secondary_gate_baseline=2500.0,
                min_fixed_update_secondary_gate_speed=7.0,
                max_fixed_update_secondary_gate_speed=15.0,
                max_fixed_prefit_rms=10.0,
                min_fixed_prefit_outliers=35,
                max_fixed_overconfidence_cov_trace=0.01,
                fixed_prefit_reset_streak=2,
                max_consec_float_reset=10,
                max_consec_nonfix_reset=10,
                max_postfix_rms=0.20,
                enable_wide_lane_ar=True,
                wide_lane_threshold=0.10,
                enable_wlnl_fallback=True,
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
            self.assertEqual(payload["rtk_max_subset_ar_drop_steps"], 18)
            self.assertEqual(payload["rtk_max_hold_divergence_m"], 5.0)
            self.assertEqual(payload["rtk_max_position_jump_m"], 20.0)
            self.assertEqual(payload["rtk_max_fixed_anchor_age_s"], 30.0)
            self.assertEqual(payload["rtk_max_fixed_doppler_consensus_m"], 10.0)
            self.assertEqual(payload["rtk_max_position_jump_min_m"], 20.0)
            self.assertEqual(payload["rtk_max_position_jump_rate_mps"], 25.0)
            self.assertEqual(payload["rtk_max_float_spp_divergence_m"], 30.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_rms_m"], 6.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_max_m"], 30.0)
            self.assertEqual(payload["rtk_max_float_prefit_residual_reset_streak"], 5)
            self.assertEqual(payload["rtk_min_float_prefit_residual_trusted_jump_m"], 8.0)
            self.assertEqual(payload["rtk_max_update_nis_per_observation"], 12.0)
            self.assertEqual(payload["rtk_max_fixed_update_nis_per_observation"], 10.0)
            self.assertEqual(payload["rtk_max_fixed_update_post_residual_rms_m"], 6.0)
            self.assertEqual(payload["rtk_max_fixed_update_gate_ratio"], 8.0)
            self.assertEqual(payload["rtk_min_fixed_update_gate_baseline_m"], 7000.0)
            self.assertEqual(payload["rtk_max_fixed_update_gate_baseline_m"], 8200.0)
            self.assertEqual(payload["rtk_min_fixed_update_gate_speed_mps"], 5.0)
            self.assertEqual(payload["rtk_max_fixed_update_gate_speed_mps"], 15.0)
            self.assertEqual(payload["rtk_max_fixed_update_secondary_gate_ratio"], 4.0)
            self.assertEqual(payload["rtk_max_fixed_prefit_residual_rms_m"], 10.0)
            self.assertEqual(payload["rtk_min_fixed_prefit_outliers"], 35)
            self.assertEqual(
                payload["rtk_max_fixed_overconfidence_covariance_trace_m2"], 0.01
            )
            self.assertEqual(payload["rtk_fixed_prefit_reset_streak"], 2)
            self.assertEqual(
                payload["rtk_min_fixed_update_secondary_gate_baseline_m"], 2000.0
            )
            self.assertEqual(
                payload["rtk_max_fixed_update_secondary_gate_baseline_m"], 2500.0
            )
            self.assertEqual(
                payload["rtk_min_fixed_update_secondary_gate_speed_mps"], 7.0
            )
            self.assertEqual(
                payload["rtk_max_fixed_update_secondary_gate_speed_mps"], 15.0
            )
            self.assertEqual(payload["rtk_max_consecutive_float_for_reset"], 10)
            self.assertEqual(payload["rtk_max_consecutive_nonfix_for_reset"], 10)
            self.assertEqual(payload["rtk_max_postfix_residual_rms_m"], 0.20)
            self.assertTrue(payload["rtk_wide_lane_ar_enabled"])
            self.assertEqual(payload["rtk_wide_lane_threshold"], 0.10)
            self.assertTrue(payload["rtk_wlnl_fallback_enabled"])
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

    @unittest.skipIf(
        os.name == "nt",
        "executes a chmod +x script stub, which Windows cannot exec directly",
    )
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
out.with_suffix(".args").write_text(" ".join(args), encoding="ascii")
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
            args_text = rtklib_pos.with_suffix(".args").read_text(encoding="ascii")
            self.assertIn("-p 2", args_text)

    def test_madocalib_ppc_rtk_config_omits_unsupported_posmode(self) -> None:
        config_path = ROOT_DIR / "scripts" / "madocalib_ppc_rtk.conf"

        text = ppc_demo.rtklib_config_text(config_path, "rtk")
        config_lines = [
            line.strip()
            for line in text.splitlines()
            if line.strip() and not line.lstrip().startswith("#")
        ]

        self.assertIn("pos1-frequency     =l1+2", text)
        self.assertFalse(any(line.startswith("pos1-posmode") for line in config_lines))


class PPCMultiCandidateSelectorTest(unittest.TestCase):
    def test_position_only_selection_preserves_baseline_status(self) -> None:
        baseline = [self.solution_epoch(0, 0.0, 3, 0.2)]
        selected = [self.solution_epoch(0, 0.1, 4, 0.2)]

        preserved, changed = ppc_multi_candidate_selector.preserve_baseline_statuses(
            selected, baseline
        )

        self.assertEqual(preserved[0].status, 3)
        self.assertEqual(changed, 1)

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

    def test_cli_help_runs_without_test_suite_import_paths(self) -> None:
        completed = subprocess.run(
            [sys.executable, str(PPC_EXPERIMENTS_DIR / "apply_ppc_multi_candidate_selector.py"), "--help"],
            cwd=ROOT_DIR,
            capture_output=True,
            text=True,
            check=False,
        )

        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("--selection-mode", completed.stdout)

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
