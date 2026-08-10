"""CLI regression cases for the RuntimeReceiverCases domain."""

from ._support import *  # noqa: F401,F403

class RuntimeReceiverCases:
    def test_replay_solves_bundled_rinex_sequence(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_replay_test_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "replay.pos"

            result = self.run_gnss(
                "replay",
                "--rover-rinex",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base-rinex",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav-rinex",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--max-epochs",
                "20",
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("summary: aligned_epochs=", result.stdout)
            self.assertIn("written_solutions=", result.stdout)
            self.assertTrue(output_path.exists())
            exported = output_path.read_text(encoding="ascii")
            self.assertIn("LibGNSS++ Position Solution", exported)
            solution_lines = [
                line for line in exported.splitlines()
                if line and not line.startswith("%")
            ]
            self.assertGreater(len(solution_lines), 0)
    def test_replay_supports_moving_base_mode(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_replay_moving_base_") as temp_dir:
            temp_root = Path(temp_dir)
            output_path = temp_root / "replay_moving_base.pos"

            result = self.run_gnss(
                "replay",
                "--rover-rinex",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base-rinex",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav-rinex",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--out",
                str(output_path),
                "--mode",
                "moving-base",
                "--max-epochs",
                "10",
                "--quiet",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode=moving-base", result.stdout)
            self.assertTrue(output_path.exists())
    def test_live_command_help_is_available(self) -> None:
        result = self.run_gnss("live", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("Usage:", result.stdout)
        self.assertIn("--rover-rtcm", result.stdout)
        self.assertIn("--rover-ubx", result.stdout)
        self.assertIn("--base-hold-seconds", result.stdout)
        self.assertIn("--mode <kinematic|moving-base>", result.stdout)
        self.assertIn("--preset <survey|low-cost|moving-base>", result.stdout)
        self.assertIn("--arfilter", result.stdout)
        self.assertIn("--no-arfilter", result.stdout)
        self.assertIn("--arfilter-margin", result.stdout)
        self.assertIn("--min-hold-count", result.stdout)
        self.assertIn("--hold-ratio-threshold", result.stdout)
        # Motion-aware kinematic gates (PR #177/#178) now reach the live runtime.
        self.assertIn("--min-full-ratio-for-subset-ar", result.stdout)
        self.assertIn("--max-pos-jump-rate", result.stdout)
        self.assertIn("--max-float-prefit-rms", result.stdout)
    def test_replay_command_help_mentions_arfilter(self) -> None:
        result = self.run_gnss("replay", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--preset <survey|low-cost|moving-base>", result.stdout)
        self.assertIn("--arfilter", result.stdout)
        self.assertIn("--no-arfilter", result.stdout)
        self.assertIn("--arfilter-margin", result.stdout)
        self.assertIn("--min-hold-count", result.stdout)
        self.assertIn("--hold-ratio-threshold", result.stdout)
        self.assertIn("--base-ubx", result.stdout)
        # Motion-aware kinematic gates (PR #177/#178) now reach the replay runtime.
        self.assertIn("--min-full-ratio-for-subset-ar", result.stdout)
        self.assertIn("--max-pos-jump-rate", result.stdout)
        self.assertIn("--max-float-prefit-rms", result.stdout)
    def test_solve_accepts_low_cost_preset_and_hold_knobs(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_solve_preset_") as temp_dir:
            output_path = Path(temp_dir) / "solve_preset.pos"
            result = self.run_gnss(
                "solve",
                "--rover",
                str(ROOT_DIR / "data" / "rover_kinematic.obs"),
                "--base",
                str(ROOT_DIR / "data" / "base_kinematic.obs"),
                "--nav",
                str(ROOT_DIR / "data" / "navigation_kinematic.nav"),
                "--preset",
                "low-cost",
                "--min-hold-count",
                "7",
                "--hold-ratio-threshold",
                "2.6",
                "--max-consec-nonfix-reset",
                "4",
                "--max-epochs",
                "5",
                "--out",
                str(output_path),
                "--no-kml",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("mode: kinematic", result.stdout)
            self.assertTrue(output_path.exists())
    @unittest.skipUnless(ros2_bag_support_available(), "ROS2 rosbag + ublox_msgs support not available")
    def test_moving_base_prepare_exports_ubx_and_reference_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_prepare_") as temp_dir:
            temp_root = Path(temp_dir)
            bag_dir = temp_root / "bag"
            build_synthetic_moving_base_rosbag(bag_dir)

            rover_ubx = temp_root / "rover.ubx"
            base_ubx = temp_root / "base.ubx"
            reference_csv = temp_root / "reference.csv"
            commercial_csv = temp_root / "commercial_receiver.csv"
            summary_json = temp_root / "summary.json"

            result = self.run_gnss(
                "moving-base-prepare",
                "--input",
                str(bag_dir),
                "--rover-ubx-out",
                str(rover_ubx),
                "--base-ubx-out",
                str(base_ubx),
                "--reference-csv",
                str(reference_csv),
                "--commercial-csv",
                str(commercial_csv),
                "--summary-json",
                str(summary_json),
                "--max-epochs",
                "2",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Prepared moving-base bag artifacts.", result.stdout)
            self.assertTrue(rover_ubx.exists())
            self.assertTrue(base_ubx.exists())
            self.assertTrue(reference_csv.exists())
            self.assertTrue(commercial_csv.exists())
            self.assertTrue(summary_json.exists())
            self.assertGreater(rover_ubx.stat().st_size, 0)
            self.assertGreater(base_ubx.stat().st_size, 0)

            summary = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(summary["rover_epochs"], 2)
            self.assertEqual(summary["matched_reference_rows"], 2)
            self.assertEqual(summary["matched_commercial_receiver_rows"], 2)
            self.assertEqual(summary["commercial_receiver_csv"], str(commercial_csv))
            self.assertEqual(summary["gps_week"], 2200)
            self.assertEqual(summary["date"], "2023-06-14")

            with reference_csv.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 2)
            self.assertEqual(rows[0]["gps_week"], "2200")
            self.assertEqual(rows[0]["gps_tow_s"], "345600.000")
            self.assertIn("baseline_n_m", rows[0])
            with commercial_csv.open(encoding="utf-8", newline="") as handle:
                commercial_rows = list(csv.DictReader(handle))
            self.assertEqual(len(commercial_rows), 2)
            self.assertEqual(commercial_rows[0]["gps_week"], "2200")
            self.assertEqual(commercial_rows[0]["solution_status"], "rtk_fixed")
            self.assertEqual(commercial_rows[0]["num_satellites"], "18")
    def test_moving_base_prepare_help_mentions_rosbag_and_ubx_exports(self) -> None:
        result = self.run_gnss("moving-base-prepare", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("ROS2 bag directory or Zenodo zip", result.stdout)
        self.assertIn("--rover-ubx-out", result.stdout)
        self.assertIn("--base-ubx-out", result.stdout)
        self.assertIn("--commercial-csv", result.stdout)
    def test_scorpion_cache_filename_preserves_zenodo_zip_name(self) -> None:
        sys_path_backup = sys.path[:]
        sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))
        sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "positioning"))
        try:
            import gnss_scorpion_moving_base_signoff as scorpion_signoff

            self.assertEqual(
                scorpion_signoff.download_cache_filename(
                    "https://zenodo.org/api/records/8083431/files/2023-06-14T174658Z.zip/content"
                ),
                "2023-06-14T174658Z.zip",
            )
        finally:
            sys.path[:] = sys_path_backup
    @unittest.skipUnless(ros2_bag_support_available(), "ROS2 rosbag + ublox_msgs support not available")
    def test_scorpion_moving_base_signoff_wraps_prepare_and_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_scorpion_moving_base_") as temp_dir:
            temp_root = Path(temp_dir)
            bag_dir = temp_root / "bag"
            build_synthetic_moving_base_rosbag(bag_dir)

            prepared_reference = temp_root / "prepared_reference.csv"
            prepared_summary = temp_root / "prepared_summary.json"
            prepare_result = self.run_gnss(
                "moving-base-prepare",
                "--input",
                str(bag_dir),
                "--reference-csv",
                str(prepared_reference),
                "--summary-json",
                str(prepared_summary),
                "--quiet",
            )
            self.assertEqual(prepare_result.returncode, 0, msg=prepare_result.stderr)

            solution_path = temp_root / "moving_base.pos"
            with prepared_reference.open(encoding="utf-8", newline="") as handle:
                rows = list(csv.DictReader(handle))
            solution_lines = ["% synthetic scorpion moving-base solution"]
            for row in rows:
                solution_lines.append(
                    " ".join(
                        [
                            row["gps_week"],
                            f"{float(row['gps_tow_s']):.3f}",
                            f"{float(row['rover_ecef_x_m']):.6f}",
                            f"{float(row['rover_ecef_y_m']):.6f}",
                            f"{float(row['rover_ecef_z_m']):.6f}",
                            "35.0",
                            "139.0",
                            "10.0",
                            "4",
                            "12",
                            "1.0",
                        ]
                    )
                )
            solution_path.write_text("\n".join(solution_lines) + "\n", encoding="ascii")

            work_dir = temp_root / "work"
            summary_json = temp_root / "scorpion_summary.json"
            result = self.run_gnss(
                "scorpion-moving-base-signoff",
                "--input",
                str(bag_dir),
                "--use-existing-solution",
                "--out",
                str(solution_path),
                "--work-dir",
                str(work_dir),
                "--summary-json",
                str(summary_json),
                "--require-matched-epochs-min",
                "2",
                "--require-fix-rate-min",
                "90",
                "--require-p95-baseline-error-max",
                "0.01",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished SCORPION moving-base sign-off.", result.stdout)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["signoff_profile"], "scorpion-moving-base")
            self.assertEqual(payload["matched_epochs"], 2)
            self.assertEqual(payload["fix_rate_pct"], 100.0)
            self.assertIsNone(payload["products_summary_json"])
            self.assertTrue(Path(payload["prepare_summary_json"]).exists())
            self.assertTrue(Path(payload["matched_csv"]).exists())
            self.assertTrue(Path(payload["commercial_receiver_csv"]).exists())
            self.assertTrue(Path(payload["commercial_receiver_matched_csv"]).exists())
            self.assertIn("commercial_receiver", payload)
            self.assertEqual(payload["commercial_receiver"]["matched_epochs"], 2)
    def test_live_signoff_summarizes_existing_log_and_enforces_realtime_gate(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_signoff_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            log_path = temp_root / "live.log"
            summary_path = temp_root / "live_summary.json"
            log_path.write_text(
                "\n".join(
                    [
                        "some debug line",
                        "summary: termination=completed rover_decoder_errors=0 base_decoder_errors=0 "
                        "aligned_epochs=3 written_solutions=3 fixed_solutions=1 "
                        "solver_wall_time_s=0.250000 solution_span_s=1.000000 "
                        "realtime_factor=4.000000 effective_epoch_rate_hz=12.000000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "live-signoff",
                "--use-existing-log",
                str(log_path),
                "--summary-json",
                str(summary_path),
                "--require-termination",
                "completed",
                "--require-written-solutions-min",
                "3",
                "--require-fixed-solutions-min",
                "1",
                "--require-realtime-factor-min",
                "1.0",
                "--require-effective-epoch-rate-min",
                "10.0",
                "--require-rover-decoder-errors-max",
                "0",
                "--require-base-decoder-errors-max",
                "0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(summary_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["metrics"]["termination"], "completed")
    def test_live_signoff_reads_config_toml(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_signoff_toml_") as temp_dir:
            temp_root = Path(temp_dir)
            log_path = temp_root / "live.log"
            summary_path = temp_root / "live_summary.json"
            config_toml = temp_root / "live_signoff.toml"
            log_path.write_text(
                "\n".join(
                    [
                        "some debug line",
                        "summary: termination=completed rover_decoder_errors=0 base_decoder_errors=0 "
                        "aligned_epochs=3 written_solutions=3 fixed_solutions=1 "
                        "solver_wall_time_s=0.250000 solution_span_s=1.000000 "
                        "realtime_factor=4.000000 effective_epoch_rate_hz=12.000000",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            config_toml.write_text(
                "\n".join(
                    [
                        "[live_signoff]",
                        f'use_existing_log = "{log_path}"',
                        f'summary_json = "{summary_path}"',
                        'require_termination = "completed"',
                        "require_written_solutions_min = 3",
                        "require_fixed_solutions_min = 1",
                        "require_realtime_factor_min = 1.0",
                        "require_effective_epoch_rate_min = 10.0",
                        "require_rover_decoder_errors_max = 0",
                        "require_base_decoder_errors_max = 0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "live-signoff",
                "--config-toml",
                str(config_toml),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["metrics"]["termination"], "completed")
            self.assertEqual(payload["metrics"]["written_solutions"], 3)
            self.assertEqual(payload["metrics"]["realtime_factor"], 4.0)
            self.assertIn("Finished live sign-off.", result.stdout)
    def test_web_serves_overview_solution_and_status_api(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_web_test_") as temp_dir:
            temp_root = Path(temp_dir)
            lib_pos = temp_root / "lib.pos"
            rtklib_pos = temp_root / "rtklib.pos"
            summary_json = temp_root / "odaiba_summary.json"
            status_json = temp_root / "receiver.status.json"
            live_summary = temp_root / "output" / "live_replay_summary.json"
            visibility_summary = temp_root / "output" / "visibility_static_summary.json"
            visibility_csv = temp_root / "output" / "visibility_static.csv"
            visibility_png = temp_root / "output" / "visibility_static.png"
            moving_base_summary = temp_root / "output" / "scorpion_moving_base_summary.json"
            moving_base_commercial = temp_root / "output" / "commercial_receiver_solution.csv"
            moving_base_commercial_matches = temp_root / "output" / "commercial_receiver_matches.csv"
            ppc_commercial = temp_root / "output" / "ppc_commercial_receiver.csv"
            ppc_commercial_matches = temp_root / "output" / "ppc_commercial_receiver_matches.csv"
            artifact_manifest = temp_root / "output" / "artifact_manifest.json"
            port_file = temp_root / "port.txt"

            lib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic libgnss++",
                        "2200 100.0 1.0 2.0 3.0 35.000000000 139.000000000 10.0 4 12 3.5",
                        "2200 101.0 2.0 3.0 4.0 35.000001000 139.000001000 10.1 3 11 2.1",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            rtklib_pos.write_text(
                "\n".join(
                    [
                        "% synthetic rtklib",
                        "2200 100.0 35.000000000 139.000000000 10.0 1 12",
                        "2200 101.0 35.000001000 139.000001000 10.1 2 11",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            summary_json.write_text(
                json.dumps(
                    {
                        "all_epochs": {
                            "libgnsspp": {"epochs": 11637, "fix_rate_pct": 8.11},
                            "rtklib": {"epochs": 8241, "fix_rate_pct": 7.22},
                        },
                        "common_epochs": {
                            "libgnsspp": {"median_h_m": 0.733387, "p95_h_m": 5.941091},
                            "rtklib": {"median_h_m": 0.703880, "p95_h_m": 27.673014},
                        },
                    }
                ),
                encoding="utf-8",
            )
            status_json.write_text(
                json.dumps(
                    {
                        "state": "running",
                        "pid": 1234,
                        "pid_running": True,
                        "uptime_seconds": 12.5,
                        "restart_count": 1,
                    }
                ),
                encoding="utf-8",
            )
            live_summary.parent.mkdir(parents=True, exist_ok=True)
            live_summary.write_text(
                json.dumps(
                    {
                        "execution_mode": "live",
                        "metrics": {
                            "termination": "completed",
                            "aligned_epochs": 3,
                            "written_solutions": 3,
                            "fixed_solutions": 1,
                            "realtime_factor": 3.5,
                            "effective_epoch_rate_hz": 12.0,
                            "rover_decoder_errors": 0,
                            "base_decoder_errors": 0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            (temp_root / "output" / "ppc_tokyo_run1_rtk_summary.json").write_text(
                json.dumps(
                    {
                        "matched_epochs": 120,
                        "fix_rate_pct": 96.67,
                        "median_h_m": 0.108,
                        "p95_h_m": 0.110,
                        "solver_wall_time_s": 12.34,
                        "realtime_factor": 1.23,
                        "effective_epoch_rate_hz": 15.67,
                        "commercial_receiver": {
                            "label": "survey_receiver",
                            "solution_pos": str(ppc_commercial),
                            "format": "csv",
                            "matched_csv": str(ppc_commercial_matches),
                            "matched_epochs": 120,
                            "valid_epochs": 120,
                            "fixed_epochs": 118,
                            "fix_rate_pct": 98.33,
                            "median_h_m": 0.095,
                            "p95_h_m": 0.105,
                            "p95_abs_up_m": 0.120,
                        },
                        "delta_vs_commercial_receiver": {
                            "matched_epochs": 0,
                            "fixed_epochs": -2,
                            "fix_rate_pct": -1.66,
                            "median_h_m": 0.013,
                            "p95_h_m": 0.005,
                            "p95_abs_up_m": -0.010,
                        },
                    }
                ),
                encoding="utf-8",
            )
            ppc_commercial.write_text(
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,solution_status,num_satellites\n",
                encoding="ascii",
            )
            ppc_commercial_matches.write_text(
                "gps_tow_s,traj_east_m,traj_north_m,traj_up_m,east_error_m,north_error_m,up_error_m,horizontal_error_m,status\n",
                encoding="ascii",
            )
            moving_base_summary.write_text(
                json.dumps(
                    {
                        "matched_epochs": 94,
                        "valid_epochs": 94,
                        "fix_rate_pct": 95.74,
                        "median_baseline_error_m": 0.042,
                        "p95_baseline_error_m": 0.101,
                        "p95_heading_error_deg": 5.85,
                        "termination": "completed",
                        "realtime_factor": 2.17,
                        "effective_epoch_rate_hz": 10.84,
                        "solution_pos": str(temp_root / "output" / "scorpion_moving_base.pos"),
                        "matched_csv": str(temp_root / "output" / "scorpion_moving_base_matches.csv"),
                        "prepare_summary_json": str(temp_root / "output" / "prepare_summary.json"),
                        "products_summary_json": str(temp_root / "output" / "products_summary.json"),
                        "plot_png": str(temp_root / "output" / "scorpion_moving_base.png"),
                        "nav_rinex": str(temp_root / "output" / "brdc0010.24n"),
                        "input_url": "https://example.com/scorpion.zip",
                        "signoff_profile": "scorpion-moving-base",
                        "commercial_receiver_csv": str(moving_base_commercial),
                        "commercial_receiver_matched_csv": str(moving_base_commercial_matches),
                        "commercial_receiver": {
                            "label": "rover_nav_pvt",
                            "solution_pos": str(moving_base_commercial),
                            "format": "csv",
                            "matched_csv": str(moving_base_commercial_matches),
                            "matched_epochs": 120,
                            "valid_epochs": 120,
                            "fixed_epochs": 120,
                            "fix_rate_pct": 100.0,
                            "median_baseline_error_m": 0.102,
                            "p95_baseline_error_m": 0.133,
                            "p95_heading_error_deg": 3.80,
                        },
                        "libgnss_vs_commercial_receiver": {
                            "matched_epochs_delta": -26,
                            "fixed_epochs_delta": -30,
                            "fix_rate_pct_delta": -4.26,
                            "median_baseline_error_m_delta": -0.060,
                            "p95_baseline_error_m_delta": -0.032,
                            "p95_heading_error_deg_delta": 2.05,
                        },
                    }
                ),
                encoding="utf-8",
            )
            moving_base_commercial.write_text(
                "gps_week,gps_tow_s,lat_deg,lon_deg,height_m,solution_status,num_satellites\n",
                encoding="ascii",
            )
            moving_base_commercial_matches.write_text(
                "gps_week,gps_tow_s,baseline_error_m,baseline_length_m,heading_error_deg,status,satellites\n",
                encoding="ascii",
            )
            (temp_root / "output" / "scorpion_moving_base.png").write_bytes(
                binascii.a2b_base64(
                    b"iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5+ymsAAAAASUVORK5CYII="
                )
            )
            (temp_root / "output" / "scorpion_moving_base_matches.csv").write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,baseline_error_m,baseline_length_m,heading_error_deg,status,satellites",
                        "2250,100.000,0.042000,1.500000,4.500000,4,12",
                        "2250,100.200,0.101000,1.510000,5.850000,4,11",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            (temp_root / "output" / "ppp_static_products_summary.json").write_text(
                json.dumps(
                    {
                        "dataset": "PPC-Dataset tokyo run1",
                        "run_dir": str(temp_root / "output" / "ppc_tokyo_run1"),
                        "reference_csv": str(temp_root / "output" / "ppc_tokyo_run1_reference.csv"),
                        "products_signoff_profile": "static",
                        "product_presets": ["igs-final", "ionex", "dcb"],
                        "fetched_product_date": "2024-01-02",
                        "ppp_solution_rate_pct": 100.0,
                        "ppp_converged": True,
                        "ppp_convergence_time_s": 285.0,
                        "mean_position_error_m": 0.12,
                        "p95_position_error_m": 0.21,
                        "max_position_error_m": 0.31,
                        "ionex_corrections": 18,
                        "dcb_corrections": 18,
                        "solution_pos": str(temp_root / "output" / "ppp_static_products.pos"),
                        "sp3": str(temp_root / "output" / "igs_static.sp3"),
                        "clk": str(temp_root / "output" / "igs_static.clk"),
                        "ionex": str(temp_root / "output" / "codg0020.24i"),
                        "dcb": str(temp_root / "output" / "CAS0MGXRAP_20240020000_01D_01D_DCB.BSX"),
                        "malib_solution_pos": str(temp_root / "output" / "malib_static.pos"),
                        "comparison_target": "MALIB",
                        "comparison_status": "better",
                        "comparison_csv": str(temp_root / "output" / "ppp_static_products_comparison.csv"),
                        "comparison_png": str(temp_root / "output" / "ppp_static_products_comparison.png"),
                        "common_epoch_pairs": 42,
                        "libgnss_minus_malib_mean_error_m": -0.05,
                        "libgnss_minus_malib_p95_error_m": -0.08,
                        "libgnss_minus_malib_max_error_m": -0.12,
                    }
                ),
                encoding="utf-8",
            )
            policy_suite_dir = temp_root / "output" / "ppc_spp_policy_suite"
            policy_suite_dir.mkdir()
            policy_report_path = policy_suite_dir / "adaptive_policy_report.json"
            policy_csv_path = policy_suite_dir / "adaptive_policy_report.csv"
            policy_suite_summary = policy_suite_dir / "adaptive_suite_summary.json"
            policy_report_path.write_text(
                json.dumps(
                    {
                        "sweep_count": 2,
                        "checks": {
                            "max_p95_delta_m": 0.0,
                            "max_positioning_drop_pct": 1.0,
                            "passed": True,
                            "failures": [],
                        },
                        "runs": [
                            {
                                "label": "tokyo_run1",
                                "selected_rate_mps": 150.0,
                                "selected_min_jump_m": 50.0,
                                "policy_positioning_rate_pct": 98.15,
                                "positioning_drop_pct": 0.97,
                                "policy_p95_h_m": 18.84,
                                "p95_h_delta_m": -1.43,
                                "jump_gate_rejected_epochs": 137,
                                "bridge_inserted_epochs": 21,
                            },
                            {
                                "label": "nagoya_run1",
                                "selected_rate_mps": 50.0,
                                "selected_min_jump_m": 40.0,
                                "policy_positioning_rate_pct": 97.69,
                                "positioning_drop_pct": 0.96,
                                "policy_p95_h_m": 9.89,
                                "p95_h_delta_m": -0.58,
                                "jump_gate_rejected_epochs": 86,
                                "bridge_inserted_epochs": 12,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            policy_csv_path.write_text("label,p95_h_delta_m\n", encoding="utf-8")
            policy_suite_summary.write_text(
                json.dumps(
                    {
                        "run_count": 2,
                        "output_dir": str(policy_suite_dir),
                        "prefix": "adaptive",
                        "rates_mps": "25,50,75,100,150,200",
                        "min_jumps_m": "10,20,30,40,50",
                        "bridge_max_gap_s": 5.0,
                        "bridge_max_anchor_speed_mps": 10.0,
                        "max_positioning_drop_pct": 1.0,
                        "max_p95_delta_m": 0.0,
                        "policy_report_json": str(policy_report_path),
                        "policy_report_csv": str(policy_csv_path),
                        "runs": [
                            {
                                "label": "tokyo_run1",
                                "reference_csv": str(temp_root / "output" / "tokyo_reference.csv"),
                                "input_pos": str(temp_root / "output" / "tokyo_input.pos"),
                                "baseline_pos": str(temp_root / "output" / "tokyo_baseline.pos"),
                                "sweep_json": str(policy_suite_dir / "tokyo_sweep.json"),
                                "sweep_csv": str(policy_suite_dir / "tokyo_sweep.csv"),
                                "filtered_pos": str(policy_suite_dir / "tokyo_policy.pos"),
                                "compare_summary_json": str(policy_suite_dir / "tokyo_compare.json"),
                                "compare_csv": str(policy_suite_dir / "tokyo_compare.csv"),
                                "compare_matched_csv": str(policy_suite_dir / "tokyo_compare_matches.csv"),
                                "compare_png": str(policy_suite_dir / "tokyo_compare.png"),
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            visibility_summary.write_text(
                json.dumps(
                    {
                        "csv": str(visibility_csv),
                        "epochs_processed": 5,
                        "epochs_with_rows": 5,
                        "rows_written": 27,
                        "unique_satellites": 9,
                        "mean_satellites_per_epoch": 5.4,
                        "max_satellites_per_epoch": 7,
                        "mean_elevation_deg": 38.2,
                        "mean_snr_dbhz": 44.7,
                    }
                ),
                encoding="utf-8",
            )
            visibility_csv.write_text(
                "\n".join(
                    [
                        "epoch_index,week,tow,satellite,system,signal,azimuth_deg,elevation_deg,snr_dbhz,has_pseudorange,has_carrier_phase,has_doppler",
                        "1,2200,100.000,G01,GPS,GPS_L1CA,45.0,30.0,42.0,1,1,0",
                        "1,2200,100.000,G02,GPS,GPS_L1CA,120.0,55.0,47.4,1,1,0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            visibility_png.write_bytes(
                binascii.a2b_base64(
                    b"iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAQAAAC1HAwCAAAAC0lEQVR42mP8/x8AAwMCAO5+ymsAAAAASUVORK5CYII="
                )
            )
            manifest_result = subprocess.run(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "artifact-manifest",
                    "--root",
                    str(temp_root),
                    "--output",
                    str(artifact_manifest),
                ],
                cwd=ROOT_DIR,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(manifest_result.returncode, 0, msg=manifest_result.stderr)
            self.assertTrue(artifact_manifest.exists())

            port = find_free_port()
            process = subprocess.Popen(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "web",
                    "--host",
                    "127.0.0.1",
                    "--port",
                    str(port),
                    "--port-file",
                    str(port_file),
                    "--root",
                    str(temp_root),
                    "--lib-pos",
                    str(lib_pos),
                    "--rtklib-pos",
                    str(rtklib_pos),
                    "--odaiba-summary",
                    str(summary_json),
                    "--rcv-status",
                    str(status_json),
                ],
                cwd=ROOT_DIR,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                bound_port = int(wait_for_file(port_file))
                with request.urlopen(f"http://127.0.0.1:{bound_port}/") as response:
                    html = response.read().decode("utf-8")
                self.assertIn("libgnss++ local web UI", html)

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/overview") as response:
                    overview = json.loads(response.read().decode("utf-8"))
                self.assertEqual(overview["odaiba_summary"]["all_epochs"]["libgnsspp"]["epochs"], 11637)
                self.assertEqual(len(overview["live_summaries"]), 1)
                self.assertEqual(overview["live_summaries"][0]["termination"], "completed")
                self.assertEqual(overview["live_summaries"][0]["realtime_factor"], 3.5)
                self.assertEqual(overview["live_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(len(overview["ppc_summaries"]), 1)
                self.assertEqual(overview["ppc_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(overview["ppc_summaries"][0]["quality_status"], "excellent")
                self.assertEqual(overview["ppc_summaries"][0]["commercial_receiver"]["label"], "survey_receiver")
                self.assertEqual(
                    overview["ppc_summaries"][0]["commercial_receiver"]["solution_pos"],
                    "output/ppc_commercial_receiver.csv",
                )
                self.assertEqual(
                    overview["ppc_summaries"][0]["commercial_receiver"]["matched_csv"],
                    "output/ppc_commercial_receiver_matches.csv",
                )
                self.assertEqual(overview["ppc_summaries"][0]["delta_vs_commercial_receiver"]["p95_h_m"], 0.005)
                self.assertEqual(overview["ppc_summaries"][0]["commercial_comparison_status"], "close")
                self.assertEqual(len(overview["moving_base_summaries"]), 1)
                self.assertEqual(overview["moving_base_summaries"][0]["runtime_status"], "realtime")
                self.assertEqual(overview["moving_base_summaries"][0]["quality_status"], "excellent")
                self.assertAlmostEqual(overview["moving_base_summaries"][0]["p95_baseline_error_m"], 0.101)
                self.assertEqual(overview["moving_base_summaries"][0]["signoff_profile"], "scorpion-moving-base")
                self.assertTrue(overview["moving_base_summaries"][0]["plot_png"].endswith("scorpion_moving_base.png"))
                self.assertTrue(overview["moving_base_summaries"][0]["matched_csv"].endswith("scorpion_moving_base_matches.csv"))
                self.assertEqual(
                    overview["moving_base_summaries"][0]["commercial_receiver"]["solution_pos"],
                    "output/commercial_receiver_solution.csv",
                )
                self.assertEqual(
                    overview["moving_base_summaries"][0]["commercial_receiver"]["matched_csv"],
                    "output/commercial_receiver_matches.csv",
                )
                self.assertEqual(
                    overview["moving_base_summaries"][0]["libgnss_vs_commercial_receiver"][
                        "p95_baseline_error_m_delta"
                    ],
                    -0.032,
                )
                self.assertEqual(overview["moving_base_summaries"][0]["commercial_comparison_status"], "worse")
                self.assertEqual(overview["moving_base_summaries"][0]["nav_rinex"], "output/brdc0010.24n")
                self.assertEqual(overview["moving_base_summaries"][0]["input_url"], "https://example.com/scorpion.zip")
                self.assertEqual(len(overview["ppp_products_summaries"]), 1)
                self.assertEqual(overview["ppp_products_summaries"][0]["profile"], "static")
                self.assertEqual(overview["ppp_products_summaries"][0]["dataset"], "PPC-Dataset tokyo run1")
                self.assertEqual(overview["ppp_products_summaries"][0]["fetched_product_date"], "2024-01-02")
                self.assertEqual(overview["ppp_products_summaries"][0]["quality_status"], "excellent")
                self.assertEqual(overview["ppp_products_summaries"][0]["ionex_corrections"], 18)
                self.assertEqual(overview["ppp_products_summaries"][0]["comparison_target"], "MALIB")
                self.assertEqual(overview["ppp_products_summaries"][0]["comparison_status"], "better")
                self.assertEqual(overview["ppp_products_summaries"][0]["common_epoch_pairs"], 42)
                self.assertEqual(overview["ppp_products_summaries"][0]["sp3"], "output/igs_static.sp3")
                self.assertEqual(overview["ppp_products_summaries"][0]["malib_solution_pos"], "output/malib_static.pos")
                self.assertEqual(overview["ppp_products_summaries"][0]["comparison_csv"], "output/ppp_static_products_comparison.csv")
                self.assertEqual(overview["ppp_products_summaries"][0]["comparison_png"], "output/ppp_static_products_comparison.png")
                self.assertEqual(len(overview["ppc_spp_policy_suites"]), 1)
                policy_suite = overview["ppc_spp_policy_suites"][0]
                self.assertEqual(policy_suite["label"], "adaptive")
                self.assertEqual(policy_suite["status"], "passed")
                self.assertEqual(policy_suite["comparison_status"], "better")
                self.assertEqual(policy_suite["policy_report"], "output/ppc_spp_policy_suite/adaptive_policy_report.json")
                self.assertEqual(policy_suite["policy_csv"], "output/ppc_spp_policy_suite/adaptive_policy_report.csv")
                self.assertEqual(policy_suite["run_count"], 2)
                self.assertEqual(policy_suite["worst_p95_h_delta_m"], -0.58)
                self.assertEqual(policy_suite["worst_positioning_drop_pct"], 0.97)
                self.assertEqual(policy_suite["runs"][0]["selected_rate_mps"], 150.0)
                self.assertEqual(
                    policy_suite["run_artifacts"][0]["compare_png"],
                    "output/ppc_spp_policy_suite/tokyo_compare.png",
                )
                self.assertEqual(len(overview["visibility_summaries"]), 1)
                self.assertEqual(overview["visibility_summaries"][0]["rows_written"], 27)
                self.assertEqual(overview["visibility_summaries"][0]["unique_satellites"], 9)
                self.assertEqual(overview["visibility_summaries"][0]["csv_path"], "output/visibility_static.csv")
                self.assertEqual(overview["visibility_summaries"][0]["png_path"], "output/visibility_static.png")
                self.assertEqual(len(overview["artifact_manifest"]), 6)
                manifest_categories = {entry["category"] for entry in overview["artifact_manifest"]}
                self.assertIn("moving-base", manifest_categories)
                self.assertIn("ppp-products", manifest_categories)
                self.assertIn("visibility", manifest_categories)
                self.assertIn("ppc-spp-policy-suite", manifest_categories)
                ppc_bundle = next(entry for entry in overview["artifact_manifest"] if entry["category"] == "ppc")
                self.assertEqual(ppc_bundle["artifacts"]["commercial_solution"], "output/ppc_commercial_receiver.csv")
                policy_suite_bundle = next(
                    entry for entry in overview["artifact_manifest"]
                    if entry["category"] == "ppc-spp-policy-suite"
                )
                self.assertEqual(policy_suite_bundle["status"], "passed")
                self.assertEqual(policy_suite_bundle["artifacts"]["policy_csv"], "output/ppc_spp_policy_suite/adaptive_policy_report.csv")
                moving_base_bundle = next(
                    entry for entry in overview["artifact_manifest"] if entry["category"] == "moving-base"
                )
                self.assertEqual(
                    moving_base_bundle["artifacts"]["commercial_matches"],
                    "output/commercial_receiver_matches.csv",
                )

                with request.urlopen(
                    f"http://127.0.0.1:{bound_port}/api/visibility?path=output/visibility_static.csv"
                ) as response:
                    visibility_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(visibility_payload["available"])
                self.assertEqual(len(visibility_payload["rows"]), 2)
                self.assertEqual(visibility_payload["rows"][0]["satellite"], "G01")

                with request.urlopen(
                    f"http://127.0.0.1:{bound_port}/api/moving-base-matches?path=output/scorpion_moving_base_matches.csv"
                ) as response:
                    moving_base_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(moving_base_payload["available"])
                self.assertEqual(moving_base_payload["path"], "output/scorpion_moving_base_matches.csv")
                self.assertGreaterEqual(len(moving_base_payload["rows"]), 1)
                self.assertIn("baseline_error_m", moving_base_payload["rows"][0])

                with request.urlopen(
                    f"http://127.0.0.1:{bound_port}/artifact?path=output/visibility_static.png"
                ) as response:
                    artifact_payload = response.read()
                self.assertGreater(len(artifact_payload), 0)

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/solution?name=libgnsspp") as response:
                    lib_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(lib_payload["available"])
                self.assertEqual(lib_payload["epoch_count"], 2)
                self.assertIn("FIXED", lib_payload["status_counts"])

                with request.urlopen(f"http://127.0.0.1:{bound_port}/api/status") as response:
                    status_payload = json.loads(response.read().decode("utf-8"))
                self.assertTrue(status_payload["available"])
                self.assertEqual(status_payload["state"], "running")
            finally:
                process.terminate()
                try:
                    process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate(timeout=5)
    def test_artifact_manifest_cli_collects_bundle_metadata(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_artifact_manifest_") as temp_dir:
            temp_root = Path(temp_dir)
            output_dir = temp_root / "output"
            output_dir.mkdir(parents=True, exist_ok=True)
            manifest_path = output_dir / "artifact_manifest.json"
            (output_dir / "ppp_static_products_summary.json").write_text(
                json.dumps(
                    {
                        "dataset": "PPC-Dataset tokyo run1",
                        "run_dir": str(output_dir / "ppc_tokyo_run1"),
                        "reference_csv": str(output_dir / "ppc_reference.csv"),
                        "products_signoff_profile": "static",
                        "fetched_product_date": "2024-01-02",
                        "ppp_solution_rate_pct": 100.0,
                        "ppp_converged": True,
                        "ppp_convergence_time_s": 285.0,
                        "p95_position_error_m": 0.21,
                        "solution_pos": str(output_dir / "ppp_static_products.pos"),
                        "sp3": str(output_dir / "igs.sp3"),
                        "clk": str(output_dir / "igs.clk"),
                        "ionex": str(output_dir / "codg0020.24i"),
                        "dcb": str(output_dir / "igs.bsx"),
                        "comparison_target": "MALIB",
                        "comparison_status": "better",
                        "comparison_csv": str(output_dir / "ppp_static_products_comparison.csv"),
                        "comparison_png": str(output_dir / "ppp_static_products_comparison.png"),
                        "common_epoch_pairs": 42,
                        "libgnss_minus_malib_mean_error_m": -0.05,
                    }
                ),
                encoding="utf-8",
            )
            (output_dir / "scorpion_moving_base_summary.json").write_text(
                json.dumps(
                    {
                        "matched_epochs": 94,
                        "fix_rate_pct": 95.74,
                        "p95_baseline_error_m": 0.101,
                        "realtime_factor": 2.17,
                        "solution_pos": str(output_dir / "scorpion_moving_base.pos"),
                        "matched_csv": str(output_dir / "scorpion_moving_base_matches.csv"),
                        "plot_png": str(output_dir / "scorpion_moving_base.png"),
                        "input_url": "https://example.com/scorpion.zip",
                        "commercial_receiver": {
                            "label": "rover_nav_pvt",
                            "solution_pos": str(output_dir / "commercial_receiver_solution.csv"),
                            "matched_csv": str(output_dir / "commercial_receiver_matches.csv"),
                            "matched_epochs": 120,
                            "fix_rate_pct": 100.0,
                            "p95_baseline_error_m": 0.133,
                        },
                        "libgnss_vs_commercial_receiver": {
                            "fix_rate_pct_delta": -4.26,
                            "p95_baseline_error_m_delta": -0.032,
                        },
                    }
                ),
                encoding="utf-8",
            )
            suite_dir = output_dir / "ppc_spp_policy_suite"
            suite_dir.mkdir()
            policy_report_path = suite_dir / "adaptive_policy_report.json"
            policy_csv_path = suite_dir / "adaptive_policy_report.csv"
            suite_summary_path = suite_dir / "adaptive_suite_summary.json"
            policy_report_path.write_text(
                json.dumps(
                    {
                        "sweep_count": 2,
                        "checks": {
                            "max_p95_delta_m": 0.0,
                            "max_positioning_drop_pct": 1.0,
                            "passed": True,
                            "failures": [],
                        },
                        "runs": [
                            {
                                "label": "tokyo_run1",
                                "selected_rate_mps": 150.0,
                                "selected_min_jump_m": 50.0,
                                "policy_positioning_rate_pct": 98.15,
                                "positioning_drop_pct": 0.97,
                                "policy_p95_h_m": 18.84,
                                "p95_h_delta_m": -1.43,
                                "jump_gate_rejected_epochs": 137,
                                "bridge_inserted_epochs": 21,
                            },
                            {
                                "label": "nagoya_run1",
                                "selected_rate_mps": 50.0,
                                "selected_min_jump_m": 40.0,
                                "policy_positioning_rate_pct": 97.69,
                                "positioning_drop_pct": 0.96,
                                "policy_p95_h_m": 9.89,
                                "p95_h_delta_m": -0.58,
                                "jump_gate_rejected_epochs": 86,
                                "bridge_inserted_epochs": 12,
                            },
                        ],
                    }
                ),
                encoding="utf-8",
            )
            policy_csv_path.write_text("label,p95_h_delta_m\n", encoding="utf-8")
            suite_summary_path.write_text(
                json.dumps(
                    {
                        "dry_run": False,
                        "run_count": 2,
                        "output_dir": str(suite_dir),
                        "prefix": "adaptive",
                        "rates_mps": "25,50,75,100,150,200",
                        "min_jumps_m": "10,20,30,40,50",
                        "bridge_max_gap_s": 5.0,
                        "bridge_max_anchor_speed_mps": 10.0,
                        "max_positioning_drop_pct": 1.0,
                        "max_p95_delta_m": 0.0,
                        "policy_report_json": str(policy_report_path),
                        "policy_report_csv": str(policy_csv_path),
                        "runs": [
                            {
                                "label": "tokyo_run1",
                                "reference_csv": str(output_dir / "tokyo_reference.csv"),
                                "input_pos": str(output_dir / "tokyo_input.pos"),
                                "baseline_pos": str(output_dir / "tokyo_baseline.pos"),
                                "sweep_json": str(suite_dir / "tokyo_sweep.json"),
                                "sweep_csv": str(suite_dir / "tokyo_sweep.csv"),
                                "filtered_pos": str(suite_dir / "tokyo_policy.pos"),
                                "compare_summary_json": str(suite_dir / "tokyo_compare.json"),
                                "compare_csv": str(suite_dir / "tokyo_compare.csv"),
                                "compare_matched_csv": str(suite_dir / "tokyo_compare_matches.csv"),
                                "compare_png": str(suite_dir / "tokyo_compare.png"),
                            }
                        ],
                    }
                ),
                encoding="utf-8",
            )
            (suite_dir / "adaptive_dry_run_suite_summary.json").write_text(
                json.dumps(
                    {
                        "dry_run": True,
                        "run_count": 2,
                        "prefix": "adaptive_dry_run",
                        "policy_report_json": str(suite_dir / "missing_policy_report.json"),
                        "runs": [],
                    }
                ),
                encoding="utf-8",
            )
            result = subprocess.run(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "artifact-manifest",
                    "--root",
                    str(temp_root),
                    "--output",
                    str(manifest_path),
                ],
                cwd=ROOT_DIR,
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(manifest_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["bundle_count"], 3)
            categories = {entry["category"] for entry in payload["bundles"]}
            self.assertEqual(categories, {"moving-base", "ppp-products", "ppc-spp-policy-suite"})
            moving_base_entry = next(entry for entry in payload["bundles"] if entry["category"] == "moving-base")
            self.assertEqual(moving_base_entry["artifacts"]["input_url"], "https://example.com/scorpion.zip")
            self.assertEqual(moving_base_entry["artifacts"]["matched_csv"], "output/scorpion_moving_base_matches.csv")
            self.assertEqual(
                moving_base_entry["artifacts"]["commercial_solution"],
                "output/commercial_receiver_solution.csv",
            )
            self.assertEqual(
                moving_base_entry["metrics"]["libgnss_vs_commercial_receiver"]["p95_baseline_error_m_delta"],
                -0.032,
            )
            ppp_entry = next(entry for entry in payload["bundles"] if entry["category"] == "ppp-products")
            self.assertEqual(ppp_entry["artifacts"]["sp3"], "output/igs.sp3")
            self.assertEqual(ppp_entry["artifacts"]["reference"], "output/ppc_reference.csv")
            self.assertEqual(ppp_entry["artifacts"]["comparison_csv"], "output/ppp_static_products_comparison.csv")
            self.assertEqual(ppp_entry["artifacts"]["comparison_png"], "output/ppp_static_products_comparison.png")
            self.assertEqual(ppp_entry["metrics"]["common_epoch_pairs"], 42)
            self.assertEqual(ppp_entry["comparison_status"], "better")
            policy_suite_entry = next(
                entry for entry in payload["bundles"]
                if entry["category"] == "ppc-spp-policy-suite"
            )
            self.assertEqual(policy_suite_entry["status"], "passed")
            self.assertEqual(policy_suite_entry["comparison_status"], "better")
            self.assertEqual(
                policy_suite_entry["artifacts"]["policy_report"],
                "output/ppc_spp_policy_suite/adaptive_policy_report.json",
            )
            self.assertEqual(
                policy_suite_entry["artifacts"]["runs"][0]["compare_png"],
                "output/ppc_spp_policy_suite/tokyo_compare.png",
            )
            self.assertEqual(policy_suite_entry["metrics"]["run_count"], 2)
            self.assertEqual(policy_suite_entry["metrics"]["worst_p95_h_delta_m"], -0.58)
            self.assertEqual(policy_suite_entry["metrics"]["worst_positioning_drop_pct"], 0.97)
            self.assertEqual(policy_suite_entry["metrics"]["runs"][0]["selected_rate_mps"], 150.0)
    def test_live_reports_missing_rtcm_source_path_clearly(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_live_missing_source_") as temp_dir:
            temp_root = Path(temp_dir)
            rover_path = temp_root / "missing_rover.rtcm3"
            base_path = temp_root / "missing_base.rtcm3"
            output_path = temp_root / "live.pos"

            result = self.run_gnss(
                "live",
                "--rover-rtcm",
                str(rover_path),
                "--base-rtcm",
                str(base_path),
                "--mode",
                "moving-base",
                "--out",
                str(output_path),
                "--quiet",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                f"Error: failed to open rover RTCM source: {rover_path}",
                result.stderr,
            )
    def test_moving_base_signoff_summarizes_existing_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_signoff_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "moving_base.pos"
            reference_csv = temp_root / "reference.csv"
            summary_path = temp_root / "moving_base_summary.json"
            plot_path = temp_root / "moving_base.png"

            solution_path.write_text(
                "\n".join(
                    [
                        "% synthetic moving-base solution fixture",
                        "2200 345600.000 3875001.100000 332002.000000 5029000.400000 35.0 139.0 10.0 4 12 1.0",
                        "2200 345601.000 3875001.200000 332002.100000 5029000.450000 35.0 139.0 10.0 4 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                        "2200,345601.0,3875000.1,332000.1,5029000.0,3875001.1,332002.1,5029000.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "moving-base-signoff",
                "--solver",
                "replay",
                "--use-existing-solution",
                "--out",
                str(solution_path),
                "--reference-csv",
                str(reference_csv),
                "--summary-json",
                str(summary_path),
                "--plot-png",
                str(plot_path),
                "--solver-wall-time-s",
                "0.5",
                "--require-valid-epochs-min",
                "2",
                "--require-matched-epochs-min",
                "2",
                "--require-fix-rate-min",
                "90",
                "--require-p95-baseline-error-max",
                "1.0",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("Finished moving-base sign-off.", result.stdout)
            self.assertTrue(summary_path.exists())
            self.assertTrue(plot_path.exists())
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["solver"], "replay")
            self.assertEqual(payload["matched_epochs"], 2)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertGreater(payload["realtime_factor"], 0.0)
            self.assertEqual(payload["plot_png"], str(plot_path))
            self.assertTrue(Path(payload["matched_csv"]).exists())
    def test_moving_base_signoff_summarizes_commercial_receiver_solution(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_commercial_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "moving_base.pos"
            commercial_path = temp_root / "commercial_receiver.csv"
            reference_csv = temp_root / "reference.csv"
            summary_path = temp_root / "moving_base_summary.json"
            matched_csv = temp_root / "moving_base_matches.csv"
            commercial_matched_csv = temp_root / "commercial_matches.csv"

            solution_path.write_text(
                "\n".join(
                    [
                        "% synthetic moving-base solution fixture",
                        "2200 345600.000 3875001.100000 332002.000000 5029000.400000 35.0 139.0 10.0 4 12 1.0",
                        "2200 345601.000 3875001.200000 332002.100000 5029000.450000 35.0 139.0 10.0 4 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            commercial_path.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m,fix_type,num_satellites",
                        "2200,345600.0,3875001.0,332002.0,5029000.5,rtk_fixed,14",
                        "2200,345601.0,3875001.1,332002.1,5029000.5,rtk_float,13",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                        "2200,345601.0,3875000.1,332000.1,5029000.0,3875001.1,332002.1,5029000.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "moving-base-signoff",
                "--solver",
                "replay",
                "--use-existing-solution",
                "--out",
                str(solution_path),
                "--reference-csv",
                str(reference_csv),
                "--summary-json",
                str(summary_path),
                "--matched-csv",
                str(matched_csv),
                "--commercial-pos",
                str(commercial_path),
                "--commercial-matched-csv",
                str(commercial_matched_csv),
                "--solver-wall-time-s",
                "0.5",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("commercial_receiver:", result.stdout)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            commercial = payload["commercial_receiver"]
            self.assertEqual(commercial["format"], "csv")
            self.assertEqual(commercial["matched_epochs"], 2)
            self.assertEqual(commercial["fixed_epochs"], 1)
            self.assertEqual(commercial["fix_rate_pct"], 50.0)
            self.assertEqual(commercial["matched_csv"], str(commercial_matched_csv))
            self.assertTrue(commercial_matched_csv.exists())
            self.assertIn("libgnss_vs_commercial_receiver", payload)
    def test_moving_base_signoff_reads_config_toml(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_signoff_toml_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "moving_base.pos"
            reference_csv = temp_root / "reference.csv"
            summary_path = temp_root / "moving_base_summary.json"
            plot_path = temp_root / "moving_base.png"
            matched_csv = temp_root / "moving_base_matches.csv"
            config_toml = temp_root / "moving_base.toml"

            solution_path.write_text(
                "\n".join(
                    [
                        "% synthetic moving-base solution fixture",
                        "2200 345600.000 3875001.100000 332002.000000 5029000.400000 35.0 139.0 10.0 4 12 1.0",
                        "2200 345601.000 3875001.200000 332002.100000 5029000.450000 35.0 139.0 10.0 4 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                        "2200,345601.0,3875000.1,332000.1,5029000.0,3875001.1,332002.1,5029000.5",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            config_toml.write_text(
                "\n".join(
                    [
                        "[moving_base_signoff]",
                        'solver = "replay"',
                        "use_existing_solution = true",
                        f'out = "{solution_path}"',
                        f'reference_csv = "{reference_csv}"',
                        f'summary_json = "{summary_path}"',
                        f'matched_csv = "{matched_csv}"',
                        f'plot_png = "{plot_path}"',
                        "solver_wall_time_s = 0.5",
                        "require_valid_epochs_min = 2",
                        "require_matched_epochs_min = 2",
                        "require_fix_rate_min = 90.0",
                        "require_p95_baseline_error_max = 1.0",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "moving-base-signoff",
                "--config-toml",
                str(config_toml),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["solver"], "replay")
            self.assertEqual(payload["matched_epochs"], 2)
            self.assertEqual(payload["fixed_epochs"], 2)
            self.assertTrue(Path(payload["matched_csv"]).exists())
    def test_moving_base_plot_renders_png_from_solution_and_reference(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_moving_base_plot_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "moving_base.pos"
            reference_csv = temp_root / "reference.csv"
            png_path = temp_root / "moving_base.png"

            solution_path.write_text(
                "\n".join(
                    [
                        "% synthetic moving-base solution fixture",
                        "2200 345600.000 3875001.100000 332002.000000 5029000.400000 35.0 139.0 10.0 4 12 1.0",
                        "2200 345601.000 3875001.200000 332002.100000 5029000.450000 35.0 139.0 10.0 4 12 1.0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )
            reference_csv.write_text(
                "\n".join(
                    [
                        "gps_week,gps_tow_s,base_ecef_x_m,base_ecef_y_m,base_ecef_z_m,rover_ecef_x_m,rover_ecef_y_m,rover_ecef_z_m",
                        "2200,345600.0,3875000.0,332000.0,5029000.0,3875001.0,332002.0,5029000.5",
                        "2200,345601.0,3875000.1,332000.1,5029000.0,3875001.1,332002.1,5029000.5",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "moving-base-plot",
                str(solution_path),
                str(reference_csv),
                str(png_path),
                "--title",
                "Synthetic moving-base",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertTrue(png_path.exists())
            self.assertIn("Saved:", result.stdout)
    def test_rcv_dry_run_and_status_snapshot(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "base_hold_seconds=0.5",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            dry_run = self.run_gnss("rcv", "--config", str(config_path), "--dry-run")
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            resolved = json.loads(dry_run.stdout)
            self.assertEqual(resolved["config"]["max_epochs"], "1")
            self.assertIn("--base-hold-seconds", resolved["command"])

            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            run_result = self.run_gnss(
                "rcv",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
            )
            self.assertNotEqual(run_result.returncode, 0)
            self.assertTrue(status_path.exists())
            status = json.loads(status_path.read_text(encoding="utf-8"))
            self.assertEqual(status["state"], "failed")
            self.assertIn("command", status)
            self.assertEqual(status["returncode"], run_result.returncode)
    def test_rcv_dry_run_accepts_rover_ubx_config(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_ubx_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        "rover_ubx=serial:///dev/ttyACM0",
                        "rover_ubx_baud=230400",
                        f"base_rtcm={temp_root / 'base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            dry_run = self.run_gnss("rcv", "--config", str(config_path), "--dry-run")
            self.assertEqual(dry_run.returncode, 0, msg=dry_run.stderr)
            resolved = json.loads(dry_run.stdout)
            self.assertEqual(resolved["config"]["rover_ubx"], "serial:///dev/ttyACM0")
            self.assertIn("--rover-ubx", resolved["command"])
            self.assertIn("--rover-ubx-baud", resolved["command"])
    def test_rcv_start_and_status_report_failed_background_run(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_start_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            start_result = self.run_gnss(
                "rcv",
                "start",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--log-out",
                str(log_path),
            )
            self.assertEqual(start_result.returncode, 0, msg=start_result.stderr)
            launched = json.loads(start_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))
            self.assertEqual(launched["log_path"], str(log_path))

            status_result = self.run_gnss(
                "rcv",
                "status",
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "5",
            )
            self.assertEqual(status_result.returncode, 0, msg=status_result.stderr)
            status = json.loads(status_result.stdout)
            self.assertEqual(status["state"], "failed")
            self.assertFalse(status["pid_running"])
            self.assertEqual(status["log_path"], str(log_path))
            self.assertGreaterEqual(status["uptime_seconds"], 0.0)
            self.assertTrue(log_path.exists())
    def test_rcv_stop_is_idempotent_when_receiver_is_not_running(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_stop_test_") as temp_dir:
            temp_root = Path(temp_dir)
            status_path = temp_root / "receiver_status.json"
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(temp_root / "receiver.conf"),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss("rcv", "stop", "--status-out", str(status_path))
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn("receiver not running", result.stdout)
    def test_rcv_status_includes_log_tail_and_restart_countdown(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_status_tail_test_") as temp_dir:
            temp_root = Path(temp_dir)
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            now = time.time()
            next_restart_at = time.strftime("%Y-%m-%dT%H:%M:%S+00:00", time.gmtime(now + 3.0))
            log_path.write_text(
                "line one\nline two\nsummary: messages=5 written_solutions=3\n",
                encoding="utf-8",
            )
            status_path.write_text(
                json.dumps(
                    {
                        "state": "restarting",
                        "pid": 999999,
                        "config_path": str(temp_root / "receiver.conf"),
                        "started_at": "2026-03-26T00:00:00+00:00",
                        "log_path": str(log_path),
                        "restart_count": 2,
                        "next_restart_at": next_restart_at,
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = self.run_gnss(
                "rcv",
                "status",
                "--status-out",
                str(status_path),
                "--tail-log-lines",
                "2",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(result.stdout)
            self.assertFalse(payload["pid_running"])
            self.assertTrue(payload["log_exists"])
            self.assertEqual(payload["log_tail"], ["line two", "summary: messages=5 written_solutions=3"])
            self.assertEqual(payload["restart_count"], 2)
            self.assertGreaterEqual(payload["next_restart_in_seconds"], 0.0)
            self.assertLessEqual(payload["next_restart_in_seconds"], 5.0)
    def test_rcv_console_accepts_scripted_status_commands(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_console_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            log_path = temp_root / "receiver.log"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            log_path.write_text("line one\nsummary: messages=5 written_solutions=3\n", encoding="utf-8")
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(config_path),
                        "log_path": str(log_path),
                        "command": ["gnss_live", "--quiet"],
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            result = subprocess.run(
                [
                    sys.executable,
                    str(DISPATCHER),
                    "rcv",
                    "console",
                    "--config",
                    str(config_path),
                    "--status-out",
                    str(status_path),
                ],
                cwd=ROOT_DIR,
                text=True,
                input="show-config\ntail 1\nquit\n",
                capture_output=True,
                check=False,
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            self.assertIn('"config_path":', result.stdout)
            self.assertIn('"log_tail": [', result.stdout)
            self.assertIn("summary: messages=5 written_solutions=3", result.stdout)
    def test_rcv_restart_without_existing_status_behaves_like_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_restart_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            restart_result = self.run_gnss(
                "rcv",
                "restart",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(restart_result.returncode, 0, msg=restart_result.stderr)
            launched = json.loads(restart_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))
    def test_rcv_reload_without_existing_status_behaves_like_start(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_reload_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )

            reload_result = self.run_gnss(
                "rcv",
                "reload",
                "--config",
                str(config_path),
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(reload_result.returncode, 0, msg=reload_result.stderr)
            launched = json.loads(reload_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["status_path"], str(status_path))
    def test_rcv_reload_resolves_config_from_existing_status(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_rcv_reload_managed_test_") as temp_dir:
            temp_root = Path(temp_dir)
            config_path = temp_root / "receiver.conf"
            status_path = temp_root / "receiver_status.json"
            output_path = temp_root / "receiver.pos"
            config_path.write_text(
                "\n".join(
                    [
                        f"rover_rtcm={temp_root / 'missing_rover.rtcm3'}",
                        f"base_rtcm={temp_root / 'missing_base.rtcm3'}",
                        f"out={output_path}",
                        "max_epochs=1",
                        "quiet=true",
                    ]
                )
                + "\n",
                encoding="utf-8",
            )
            status_path.write_text(
                json.dumps(
                    {
                        "state": "completed",
                        "pid": 999999,
                        "config_path": str(config_path),
                    }
                )
                + "\n",
                encoding="utf-8",
            )

            reload_result = self.run_gnss(
                "rcv",
                "reload",
                "--status-out",
                str(status_path),
                "--wait-seconds",
                "0.1",
            )
            self.assertEqual(reload_result.returncode, 0, msg=reload_result.stderr)
            launched = json.loads(reload_result.stdout)
            self.assertEqual(launched["state"], "starting")
            self.assertEqual(launched["config_path"], str(config_path))
            self.assertEqual(launched["status_path"], str(status_path))
    @unittest.skipUnless(
        os.path.exists("gnssplusplus_thesis_ws/data/clas/claslib/data/0627239Q.obs"),
        "CLAS regression data not available"
    )
    def test_clas_ppp_regression_cm_accuracy(self) -> None:
        """Regression test: CLAS PPP must achieve sub-20cm (last100) with 1-hour data."""
        import numpy as np
        data_dir = "gnssplusplus_thesis_ws/data/clas/claslib/data"
        with tempfile.TemporaryDirectory(prefix="gnss_clas_regression_") as temp_dir:
            temp_root = Path(temp_dir)
            l6_path = f"{data_dir}/2019239Q.l6"
            expanded_csv = temp_root / "expanded.csv"
            sys_path_backup = sys.path[:]
            sys.path.insert(0, str(ROOT_DIR / "apps" / "commands"))
            sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "receivers"))
            sys.path.insert(0, str(ROOT_DIR / "apps" / "commands" / "positioning"))
            try:
                from gnss_clas_ppp import expand_qzss_l6_source
                expand_qzss_l6_source(l6_path, 2068, expanded_csv)
            finally:
                sys.path[:] = sys_path_backup

            output_path = temp_root / "clas_regression.pos"
            result = self.run_gnss(
                "ppp",
                "--obs", f"{data_dir}/0627239Q.obs",
                "--nav", f"{data_dir}/sept_2019239.nav",
                "--ssr", str(expanded_csv),
                "--out", str(output_path),
                "--static",
                "--no-ionosphere-free",
                "--estimate-ionosphere",
                "--clas-osr",
                "--clas-epoch-policy", "strict-osr",
                "--max-epochs", "3600",
            )
            self.assertEqual(result.returncode, 0, msg=result.stderr)

            ref = np.array([-3957235.4242195, 3310368.2649278, 3737529.7667316])
            errors = []
            with output_path.open() as f:
                for line in f:
                    if line.startswith("%") or not line.strip():
                        continue
                    parts = line.split()
                    if len(parts) < 10:
                        continue
                    pos = np.array([float(parts[2]), float(parts[3]), float(parts[4])])
                    errors.append(float(np.linalg.norm(pos - ref)))
            self.assertGreater(len(errors), 3500, "Expected ~3600 epochs")

            errors_arr = np.array(errors)
            last100_mean = float(errors_arr[-100:].mean())
            min_error = float(errors_arr.min())

            self.assertLess(last100_mean, 0.20, f"Last100 mean {last100_mean:.4f}m exceeds 0.20m")
            self.assertLess(min_error, 0.05, f"Min error {min_error:.6f}m exceeds 0.05m")
