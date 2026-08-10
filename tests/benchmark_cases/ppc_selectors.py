"""PPC selector and coverage benchmark test cases."""

from ._support import *  # noqa: F401,F403

__all__ = [
    "PPCCoverageMatrixTest",
    "PPCResidualResetSweepAnalysisTest",
    "PPCRealtimeGuardSweepTest",
    "PPCProfileSegmentDeltaTest",
    "PPCSegmentSelectorSweepTest",
    "PPCSegmentSelectorLeaveOneRunOutTest",
    "PPCDualProfileSelectorTest",
    "PPCDualProfileSelectorMatrixTest",
    "PPCDualProfileSelectorDriverTest",
    "PPCIMUCoverageTest",
    "PPCIMUBridgeTargetsTest",
    "PPCCVDropoutBridgeMatrixTest",
    "PPCIMUDropoutBridgeMatrixTest",
    "PPCCoverageReadmeUpdateTest",
]


class PPCCoverageMatrixTest(unittest.TestCase):
    def test_default_profile_uses_guarded_deep_partial_ar(self) -> None:
        with mock.patch.object(
            sys,
            "argv",
            ["gnss_ppc_coverage_matrix.py", "--dataset-root", "data/PPC-Dataset"],
        ):
            args = ppc_coverage_matrix.parse_args()

        self.assertEqual(args.max_subset_ar_drop_steps, 18)

    def test_parse_args_loads_config_toml_profile_and_allows_cli_overrides(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_coverage_config_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
            config_toml = temp_root / "coverage.toml"
            config_toml.write_text(
                "\n".join(
                    [
                        "[ppc_coverage_matrix]",
                        f'dataset_root = "{dataset_root.as_posix()}"',
                        'output_dir = "output/ppc_sigma_profile_runtime_demote_nis2_ratio4"',
                        'summary_json = "output/ppc_sigma_profile_runtime_demote_nis2_ratio4/summary.json"',
                        'markdown_output = "output/ppc_sigma_profile_runtime_demote_nis2_ratio4/table.md"',
                        "max_epochs = -1",
                        'preset = "low-cost"',
                        "ratio = 2.8",
                        "carrier_phase_sigma = 0.001",
                        "max_postfix_rms = 0.20",
                        "max_consec_float_reset = 10",
                        "max_subset_ar_drop_steps = 18",
                        "adaptive_dynamic_slip_thresholds = true",
                        "adaptive_dynamic_slip_nonfix_count = 25",
                        "max_pos_jump = 5.0",
                        "max_pos_jump_min = 5.0",
                        "max_pos_jump_rate = 25.0",
                        "demote_fixed_status_nis_per_obs = 2.0",
                        "demote_fixed_status_max_ratio = 4.0",
                        "demote_fixed_status_min_satellites = 9",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            with mock.patch.object(
                sys,
                "argv",
                [
                    "gnss_ppc_coverage_matrix.py",
                    "--config-toml",
                    str(config_toml),
                    "--max-epochs",
                    "20",
                ],
            ):
                args = ppc_coverage_matrix.parse_args()

            self.assertEqual(args.dataset_root, dataset_root)
            self.assertEqual(
                args.output_dir,
                Path("output/ppc_sigma_profile_runtime_demote_nis2_ratio4"),
            )
            self.assertEqual(
                args.summary_json,
                Path("output/ppc_sigma_profile_runtime_demote_nis2_ratio4/summary.json"),
            )
            self.assertEqual(
                args.markdown_output,
                Path("output/ppc_sigma_profile_runtime_demote_nis2_ratio4/table.md"),
            )
            self.assertEqual(args.max_epochs, 20)
            self.assertEqual(args.preset, "low-cost")
            self.assertEqual(args.ratio, 2.8)
            self.assertEqual(args.carrier_phase_sigma, 0.001)
            self.assertEqual(args.max_postfix_rms, 0.20)
            self.assertEqual(args.max_consec_float_reset, 10)
            self.assertEqual(args.max_subset_ar_drop_steps, 18)
            self.assertTrue(args.adaptive_dynamic_slip_thresholds)
            self.assertEqual(args.adaptive_dynamic_slip_nonfix_count, 25)
            self.assertEqual(args.max_pos_jump, 5.0)
            self.assertEqual(args.max_pos_jump_min, 5.0)
            self.assertEqual(args.max_pos_jump_rate, 25.0)
            self.assertEqual(args.demote_fixed_status_nis_per_obs, 2.0)
            self.assertEqual(args.demote_fixed_status_max_ratio, 4.0)
            self.assertEqual(args.demote_fixed_status_min_satellites, 9)

    def test_tracked_sigma_demote_profile_config_parses(self) -> None:
        config_toml = ROOT_DIR / "configs" / "benchmarks" / "ppc_sigma_demote_nis2_ratio4.toml"

        with mock.patch.object(
            sys,
            "argv",
            [
                "gnss_ppc_coverage_matrix.py",
                "--config-toml",
                str(config_toml),
            ],
        ):
            args = ppc_coverage_matrix.parse_args()

        self.assertEqual(args.dataset_root, Path("data/PPC-Dataset"))
        self.assertEqual(args.ratio, 2.8)
        self.assertEqual(args.carrier_phase_sigma, 0.001)
        self.assertEqual(args.demote_fixed_status_nis_per_obs, 2.0)
        self.assertEqual(args.demote_fixed_status_max_ratio, 4.0)
        self.assertEqual(
            args.summary_json,
            Path("output/ppc_sigma_profile_runtime_demote_nis2_ratio4/summary.json"),
        )

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
                elevation_mask_deg=22.5,
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
                rtk_snr_weighting=True,
                rtk_snr_reference_dbhz=44.0,
                rtk_snr_max_variance_scale=16.0,
                rtk_snr_min_baseline=7000.0,
                cycle_slip_threshold=0.08,
                doppler_slip_threshold=0.15,
                code_slip_threshold=4.0,
                strict_dynamic_slip_thresholds=True,
                adaptive_dynamic_slip_thresholds=True,
                adaptive_dynamic_slip_nonfix_count=2,
                adaptive_dynamic_slip_hold_epochs=8,
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
            self.assertIn("--elevation-mask-deg", command)
            self.assertIn("22.5", command)
            self.assertIn("2.4", command)
            self.assertIn("--max-subset-ar-drop-steps", command)
            self.assertIn("18", command)
            self.assertIn("--max-hold-div", command)
            self.assertIn("5.0", command)
            self.assertIn("--max-pos-jump", command)
            self.assertIn("20.0", command)
            self.assertIn("--max-fixed-anchor-age", command)
            self.assertIn("30.0", command)
            self.assertIn("--max-fixed-doppler-consensus", command)
            self.assertIn("10.0", command)
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
            self.assertIn("--max-fixed-prefit-rms", command)
            self.assertIn("--min-fixed-prefit-outliers", command)
            self.assertIn("--max-fixed-overconfidence-cov-trace", command)
            self.assertIn("0.01", command)
            self.assertIn("--fixed-prefit-reset-streak", command)
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
            self.assertIn("--min-fixed-update-gate-speed", command)
            self.assertIn("5.0", command)
            self.assertIn("--max-fixed-update-gate-speed", command)
            self.assertIn("15.0", command)
            self.assertIn("--max-fixed-update-secondary-gate-ratio", command)
            self.assertIn("4.0", command)
            self.assertIn("--min-fixed-update-secondary-gate-baseline", command)
            self.assertIn("2000.0", command)
            self.assertIn("--max-fixed-update-secondary-gate-baseline", command)
            self.assertIn("2500.0", command)
            self.assertIn("--min-fixed-update-secondary-gate-speed", command)
            self.assertIn("7.0", command)
            self.assertIn("--max-fixed-update-secondary-gate-speed", command)
            self.assertIn("15.0", command)
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
                max_subset_ar_drop_steps=None,
                max_hold_div=None,
                max_pos_jump=None,
                max_pos_jump_min=None,
                max_pos_jump_rate=None,
                max_consec_float_reset=None,
                max_consec_nonfix_reset=None,
                max_postfix_rms=None,
                enable_wide_lane_ar=False,
                wide_lane_threshold=None,
                enable_wlnl_fallback=False,
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
                            "ppc_score_3d_50cm_ref_pct": 0.0,
                            "p95_h_m": 31.13,
                            "max_h_m": 52.0,
                            "solver_wall_time_s": 0.5,
                            "realtime_factor": 20.0,
                            "effective_epoch_rate_hz": 100.0,
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

            self.assertEqual(payload["summary_schema"], ppc_coverage_matrix.SUMMARY_SCHEMA)
            ppc_coverage_matrix.validate_matrix_payload_schema(payload)
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
            self.assertIsNone(payload["max_subset_ar_drop_steps"])
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
            self.assertFalse(payload["enable_wlnl_fallback"])
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

            invalid_payload = json.loads(json.dumps(payload))
            del invalid_payload["runs"][0]["metrics"]["p95_h_m"]
            with self.assertRaises(SystemExit) as schema_context:
                ppc_coverage_matrix.validate_matrix_payload_schema(invalid_payload)
            self.assertIn("metrics: missing `p95_h_m`", str(schema_context.exception))


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
                use_existing_matrices=False,
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

    def test_run_profile_can_load_existing_matrix(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_realtime_existing_") as temp_dir:
            temp_root = Path(temp_dir)
            matrix_dir = temp_root / "out" / "coverage"
            matrix_dir.mkdir(parents=True)
            matrix_payload = {
                "runs": [
                    {
                        "key": "tokyo_run1",
                        "metrics": {
                            "ppc_official_score_distance_m": 1.0,
                            "ppc_official_total_distance_m": 2.0,
                        },
                    }
                ]
            }
            matrix_dir.joinpath("matrix.json").write_text(
                json.dumps(matrix_payload),
                encoding="utf-8",
            )
            args = argparse.Namespace(
                dataset_root=temp_root / "PPC-Dataset",
                output_dir=temp_root / "out",
                summary_json=temp_root / "summary.json",
                markdown_output=None,
                max_epochs=-1,
                match_tolerance_s=0.25,
                preset="low-cost",
                rtklib_root=None,
                rtklib_bin=None,
                rtklib_config=ROOT_DIR / "scripts" / "rtklib_odaiba.conf",
                use_existing_solutions=False,
                use_existing_matrices=True,
                require_positioning_delta_min=None,
                require_official_score_delta_min=None,
                require_p95_h_delta_max=None,
                require_solver_wall_time_max=None,
                require_realtime_factor_min=None,
                require_effective_epoch_rate_min=None,
            )

            matrix_json, matrix_md, payload = ppc_realtime_guard_sweep.run_profile(
                args,
                ppc_realtime_guard_sweep.GuardProfile("coverage", ("--ratio", "2.4")),
            )

            self.assertEqual(matrix_json, matrix_dir / "matrix.json")
            self.assertEqual(matrix_md, matrix_dir / "matrix.md")
            self.assertEqual(payload["runs"][0]["key"], "tokyo_run1")

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
        self.assertEqual(payload["best_safe_profile"]["name"], "fixed_update_guard")
        self.assertEqual(payload["profiles"][1]["delta_vs_baseline_score_pct"], 3.0)
        self.assertEqual(payload["profiles"][1]["min_realtime_factor"], 3.5)
        self.assertFalse(payload["profiles"][1]["truth_diagnostics"]["available"])
        self.assertIn("fixed_update_guard", markdown)
        self.assertIn("Best no-worse-Wrong-FIX profile", markdown)
        self.assertIn("3.500", markdown)
        self.assertIn("--max-fixed-update-nis-per-obs", markdown)

    def test_truth_diagnostics_classifies_wrong_fixes(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppc_realtime_truth_") as temp_dir:
            temp_root = Path(temp_dir)
            dataset_root = temp_root / "PPC-Dataset"
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
            solution_dir = temp_root / "solution"
            solution_dir.mkdir()
            solution_dir.joinpath("tokyo_run1.pos").write_text(
                "\n".join(
                    [
                        "% synthetic",
                        "2300 10.000 1000.000 2000.000 3000.000 0 0 0 4 12 1.0",
                        "2300 10.200 1011.000 2001.000 3001.000 0 0 0 4 12 1.0",
                        "2300 10.400 1002.100 2002.000 3002.000 0 0 0 3 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            diagnostics = ppc_realtime_guard_sweep.truth_diagnostics(
                dataset_root,
                solution_dir,
                ["tokyo_run1"],
            )

            self.assertTrue(diagnostics["available"])
            self.assertEqual(diagnostics["matched"], 3)
            self.assertEqual(diagnostics["fix_ok"], 1)
            self.assertEqual(diagnostics["fix_wrong"], 1)
            self.assertEqual(diagnostics["float_ok"], 1)
            self.assertEqual(diagnostics["fix_wrong_rate_pct"], 50.0)


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

    def test_common_metrics_separate_correct_and_wrong_fixed_epochs(self) -> None:
        reference = [self.reference_epoch(index) for index in range(2)]
        solution = [
            self.solution_epoch(0, 0.1, 4, None),
            self.solution_epoch(1, 11.0, 4, None),
        ]

        metrics = ppc_metrics.summarize_solution_epochs(
            reference,
            solution,
            fixed_status=4,
            label="fixed-quality-audit",
            match_tolerance_s=0.25,
            solver_wall_time_s=None,
        )

        self.assertEqual(metrics["fixed_epochs"], 2)
        self.assertEqual(metrics["correct_fix_epochs"], 1)
        self.assertEqual(metrics["wrong_fix_epochs"], 1)
        self.assertEqual(metrics["wrong_fix_rate_pct"], 50.0)
        self.assertEqual(metrics["correct_fix_matched_pct"], 50.0)
        self.assertEqual(metrics["correct_fix_ref_pct"], 50.0)


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

class PPCCoverageReadmeUpdateTest(unittest.TestCase):
    def test_default_target_keeps_generated_table_in_benchmarks(self) -> None:
        self.assertEqual(
            ppc_coverage_readme.DEFAULT_TARGETS,
            (ppc_coverage_readme.ROOT_DIR / "docs" / "benchmarks.md",),
        )

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
