"""CLI regression cases for the CLISurfaceCases domain."""

from ._support import *  # noqa: F401,F403

class CLISurfaceCases:
    def test_odaiba_benchmark_help_is_available(self) -> None:
        result = self.run_gnss("odaiba-benchmark", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--summary-json", result.stdout)
        self.assertIn("--require-all-epochs-min", result.stdout)
    def test_odaiba_scan_help_is_available(self) -> None:
        result = self.run_gnss("odaiba-scan", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("--window-size", result.stdout)
        self.assertIn("--output-csv", result.stdout)
    def test_ppp_help_hides_madocalib_oracle_options(self) -> None:
        result = self.run_gnss("ppp", "--help")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertNotIn("--madocalib-bridge", result.stdout)
        self.assertNotIn("--madocalib-trace-ar", result.stdout)
        self.assertNotIn("--madocalib-l6", result.stdout)
        self.assertNotIn("--madocalib-mdciono", result.stdout)
        self.assertNotIn("--madocalib-profile", result.stdout)
        self.assertIn("--madoca-l6d-shadow", result.stdout)
        self.assertIn("--madoca-materialization-dump", result.stdout)
        self.assertIn("--madoca-materialization-dump-only", result.stdout)
    def test_ppp_cli_rejects_madocalib_oracle_selector(self) -> None:
        result = self.run_gnss(
            "ppp",
            "--obs",
            "missing.obs",
            "--nav",
            "missing.nav",
            "--out",
            "unused.pos",
            "--madocalib-bridge",
            "--madocalib-profile",
            "unknown",
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn(
            "unknown or incomplete argument: --madocalib-bridge",
            result.stderr,
        )
    def test_ppp_cli_accepts_more_than_three_hourly_madoca_l6d_inputs(self) -> None:
        result = self.run_gnss(
            "ppp",
            "--obs",
            "missing.obs",
            "--nav",
            "missing.nav",
            "--out",
            "unused.pos",
            "--ar-method",
            "per-freq",
            "--madoca-l6",
            "missing-l6e.l6",
            *[
                option
                for hour in "ABCD"
                for option in ("--madoca-l6d", f"2025091{hour}.200.l6")
            ],
        )

        self.assertNotEqual(result.returncode, 0)
        self.assertNotIn("accepts at most three files", result.stderr)
    def test_ppp_cli_rejects_madoca_materialization_dump_only_without_dump_path(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_ppp_madoca_materialization_dump_only_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            obs_path, sp3_path, _, _ = build_synthetic_ppp_inputs(temp_root)

            result = self.run_gnss(
                "ppp",
                "--static",
                "--obs",
                str(obs_path),
                "--sp3",
                str(sp3_path),
                "--madoca-materialization-dump-only",
            )

            self.assertNotEqual(result.returncode, 0)
            self.assertIn(
                "--madoca-materialization-dump-only requires --madoca-materialization-dump",
                result.stderr,
            )
    def test_public_rtk_benchmarks_cli_lists_matrix(self) -> None:
        result = self.run_gnss("public-rtk-benchmarks", "--format", "json")
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        payload = json.loads(result.stdout)
        profiles = {
            profile["profile_id"]: profile
            for profile in payload["public_rtk_benchmarks"]
        }

        self.assertEqual(profiles["urban-nav-tokyo"]["status"], "wired-path-overrides")
        self.assertEqual(profiles["ppc-dataset"]["status"], "primary-public-rtk-signoff")
        self.assertIn("Septentrio mosaic-X5", profiles["ppc-dataset"]["receiver_artifacts"])
        self.assertEqual(profiles["smartloc"]["status"], "receiver-fix-signoff")
        self.assertIn("not the Trimble RTK engine", profiles["urban-nav-tokyo"]["caveat"])
    def test_smartloc_adapter_cli_exports_reference_and_receiver_csv(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_adapter_cli_") as temp_dir:
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
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "smartloc-adapter",
                "--nav-posllh",
                str(nav_posllh),
                "--reference-csv",
                str(reference_csv),
                "--receiver-csv",
                str(receiver_csv),
                "--summary-json",
                str(summary_json),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["adapter_status"], "receiver_csv_adapter")
            self.assertEqual(payload["epochs"], 1)
            self.assertTrue(reference_csv.exists())
            self.assertTrue(receiver_csv.exists())
            self.assertIn("Finished smartLoc adapter export.", result.stdout)
    def test_smartloc_adapter_cli_downloads_input_url_to_cache(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_adapter_url_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_zip = temp_root / "smartloc_nav_fixture.zip"
            cache_dir = temp_root / "cache"
            reference_csv = temp_root / "reference.csv"
            receiver_csv = temp_root / "receiver.csv"
            summary_json = temp_root / "summary.json"
            nav_text = (
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon Cov) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat Cov) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];GPS time of week of the navigation epoch (iTOW) [ms];Longitude (lon) [deg];Latitude (lat) [deg];Height above ellipsoid (height) [m];Height above mean sea level (hMSL) [m];Horizontal accuracy estimate (hAcc) [m];Vertical accuracy estimate (vAcc) [m]",
                        "1900;126641.5;13.373657763;0.0;52.504560275;0.0;76.004611;0.0;1.24;0.0;0.9;0.0;5.7;0.0;0.002;0.0;126641500;13.3736578;52.5045603;76.104;38.043;0.547;-1",
                    ]
                )
                + "\n"
            )
            with zipfile.ZipFile(source_zip, "w") as archive:
                archive.writestr("NAV-POSLLH.csv", nav_text)

            result = self.run_gnss(
                "smartloc-adapter",
                "--input-url",
                source_zip.as_uri(),
                "--download-cache-dir",
                str(cache_dir),
                "--reference-csv",
                str(reference_csv),
                "--receiver-csv",
                str(receiver_csv),
                "--summary-json",
                str(summary_json),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            cached_zip = cache_dir / source_zip.name
            self.assertTrue(cached_zip.exists())
            self.assertEqual(payload["input_url"], source_zip.as_uri())
            self.assertEqual(payload["downloaded_input"], str(cached_zip))
            self.assertEqual(payload["source_path"], str(cached_zip))
    def test_smartloc_adapter_cli_exports_rawx_rinex(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_rawx_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            rawx = temp_root / "RXM-RAWX.csv"
            raw_csv = temp_root / "rawx.csv"
            obs_rinex = temp_root / "rover.obs"
            summary_json = temp_root / "summary.json"
            rawx.write_text(
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];Measurement time of week (rcvTow) [s];GPS week number (week) [weeks];GPS leap seconds (leapS) [s];Number of measurements to follow (numMeas) [];Receiver tracking status (recStat) [];Pseudorange measurement (prMes) [m];Carrier phase measurement (cpMes) [cycles];Doppler measurement (doMes) [Hz];GNSS identifier (gnssId) [];Satellite identifier (svId) [];Frequency slot - only Glonass (freqId) [];Carrier phase locktime counter (locktime) [ms];Carrier-to-noise density ratio (cno) [dbHz];Estimated pseudorange measurement standard deviation (prStdev) [m];Estimated carrier phase measurement standard deviation (cpStdev) [cycles];Estimated Doppler measurement standard deviation (doStdev) [Hz];Tracking status (trkStat) [];NLOS (0 == no, 1 == yes, # == No Information)",
                        "1900;126641.5;13.0;0.0;52.0;0.0;76.0;0.0;1.0;0.0;0.0;0.0;5.0;0.0;0.0;0.0;126641.5;1900;17;1;1;19834597.871;104231506.047;222.1658;GPS;12;0;64500;50;0.32;0.004;0.128;15;0",
                    ]
                )
                + "\n",
                encoding="ascii",
            )

            result = self.run_gnss(
                "smartloc-adapter",
                "--rawx",
                str(rawx),
                "--raw-csv",
                str(raw_csv),
                "--obs-rinex",
                str(obs_rinex),
                "--summary-json",
                str(summary_json),
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            payload = json.loads(summary_json.read_text(encoding="utf-8"))
            self.assertEqual(payload["adapter_status"], "rawx_rinex_adapter")
            self.assertEqual(payload["raw_epochs"], 1)
            self.assertTrue(raw_csv.exists())
            self.assertTrue(obs_rinex.exists())
            self.assertIn("raw_observations:", result.stdout)
    def test_smartloc_signoff_cli_writes_receiver_fix_summary(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_signoff_cli_") as temp_dir:
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

            result = self.run_gnss(
                "smartloc-signoff",
                "--nav-posllh",
                str(nav_posllh),
                "--rawx",
                str(rawx),
                "--output-dir",
                str(output_dir),
                "--require-matched-epochs-min",
                "2",
                "--require-p95-h-max",
                "1.0",
                "--require-raw-epochs-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary_path = output_dir / "smartloc_signoff_summary.json"
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            self.assertEqual(payload["receiver_fix"]["matched_epochs"], 2)
            self.assertEqual(payload["raw_adapter"]["raw_epochs"], 1)
            self.assertEqual(payload["solver_preflight"]["status"], "blocked")
            self.assertIn(
                "missing base-station observation stream in smartLoc input",
                payload["solver_preflight"]["rtk_blockers"],
            )
            self.assertFalse(payload["solver_signoff_available"])
            self.assertIn("Finished smartLoc sign-off.", result.stdout)
    def test_smartloc_signoff_cli_downloads_input_url_to_cache(self) -> None:
        with tempfile.TemporaryDirectory(prefix="gnss_smartloc_signoff_url_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            source_zip = temp_root / "berlin1_fixture.zip"
            cache_dir = temp_root / "cache"
            output_dir = temp_root / "out"
            nav_text = (
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon Cov) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat Cov) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];GPS time of week of the navigation epoch (iTOW) [ms];Longitude (lon) [deg];Latitude (lat) [deg];Height above ellipsoid (height) [m];Height above mean sea level (hMSL) [m];Horizontal accuracy estimate (hAcc) [m];Vertical accuracy estimate (vAcc) [m]",
                        "1900;126641.5;13.373657763;0.0;52.504560275;0.0;76.004611;0.0;1.24;0.0;0.9;0.0;5.7;0.0;0.002;0.0;126641500;13.3736578;52.5045603;76.104;38.043;0.547;-1",
                        "1900;126641.7;13.373662801;0.0;52.504570098;0.0;76.010967;0.0;1.23;0.0;0.8;0.0;5.8;0.0;-0.007;0.0;126641700;13.3736629;52.5045702;76.111;38.035;0.547;-1",
                    ]
                )
                + "\n"
            )
            raw_text = (
                "\n".join(
                    [
                        "GPSWeek [weeks];GPSSecondsOfWeek [s];Longitude (GT Lon) [deg];Longitude Cov (GT Lon) [deg];Latitude (GT Lat) [deg];Latitude Cov (GT Lat) [deg];Height above ellipsoid (GT Height) [m];Height above ellipsoid Cov (GT Height Cov) [m];Heading (0 = East, counterclockwise) - (GT Heading) [rad];Heading Cov (0 = East, counterclockwise) - (GT Heading Cov) [rad];Acceleration (GT Acceleration) [ms^2];Acceleration Cov (GT Acceleration Cov) [ms^2];Velocity (GT Velocity) [m/s];Velocity Cov (GT Velocity Cov) [m/s];Yaw-Rate (GT Yaw-rate) [rad/s];Yaw-Rate Cov (GT Yaw-rate Cov) [rad/s];Measurement time of week (rcvTow) [s];GPS week number (week) [weeks];GPS leap seconds (leapS) [s];Number of measurements to follow (numMeas) [];Receiver tracking status (recStat) [];Pseudorange measurement (prMes) [m];Carrier phase measurement (cpMes) [cycles];Doppler measurement (doMes) [Hz];GNSS identifier (gnssId) [];Satellite identifier (svId) [];Frequency slot - only Glonass (freqId) [];Carrier phase locktime counter (locktime) [ms];Carrier-to-noise density ratio (cno) [dbHz];Estimated pseudorange measurement standard deviation (prStdev) [m];Estimated carrier phase measurement standard deviation (cpStdev) [cycles];Estimated Doppler measurement standard deviation (doStdev) [Hz];Tracking status (trkStat) [];NLOS (0 == no, 1 == yes, # == No Information)",
                        "1900;126641.5;13.0;0.0;52.0;0.0;76.0;0.0;1.0;0.0;0.0;0.0;5.0;0.0;0.0;0.0;126641.5;1900;17;1;1;19834597.871;104231506.047;222.1658;GPS;12;0;64500;50;0.32;0.004;0.128;15;0",
                    ]
                )
                + "\n"
            )
            with zipfile.ZipFile(source_zip, "w") as archive:
                archive.writestr("berlin/scenario1/NAV-POSLLH.csv", nav_text)
                archive.writestr("berlin/scenario1/RXM-RAWX.csv", raw_text)
                archive.writestr("berlin/scenario1/gbm19001.sp3.Z", "synthetic precise orbit\n")

            result = self.run_gnss(
                "smartloc-signoff",
                "--input-url",
                source_zip.as_uri(),
                "--download-cache-dir",
                str(cache_dir),
                "--output-dir",
                str(output_dir),
                "--require-matched-epochs-min",
                "2",
                "--require-raw-epochs-min",
                "1",
            )

            self.assertEqual(result.returncode, 0, msg=result.stderr)
            summary_path = output_dir / "smartloc_signoff_summary.json"
            payload = json.loads(summary_path.read_text(encoding="utf-8"))
            cached_zip = cache_dir / source_zip.name
            self.assertTrue(cached_zip.exists())
            self.assertEqual(payload["input_url"], source_zip.as_uri())
            self.assertEqual(payload["downloaded_input"], str(cached_zip))
            self.assertEqual(payload["adapter"]["source_path"], str(cached_zip))
            self.assertEqual(payload["receiver_fix"]["matched_epochs"], 2)
            self.assertEqual(
                payload["solver_preflight"]["precise_orbit_candidates"],
                ["berlin/scenario1/gbm19001.sp3.Z"],
            )
            self.assertFalse(payload["solver_preflight"]["ppp_smoke_available"])
            self.assertIn(
                "missing precise CLK clock product",
                payload["solver_preflight"]["ppp_blockers"],
            )
    def test_ros2_solution_node_runs_via_dispatcher_when_built(self) -> None:
        if not ros2_solution_node_exists():
            self.skipTest("gnss_solution_node is not built in this environment")
        with tempfile.TemporaryDirectory(prefix="gnss_ros2_dispatcher_cli_") as temp_dir:
            temp_root = Path(temp_dir)
            solution_path = temp_root / "sample.pos"
            solution_path.write_text(
                "% synthetic solution\n"
                "1316 518400.0 -3978242.0 3382841.0 3649903.0 35.0 139.0 10.0 4 9 1.0\n"
                "1316 518430.0 -3978243.0 3382840.0 3649902.0 35.0 139.0 10.0 6 10 2.5\n",
                encoding="ascii",
            )
            result = self.run_gnss(
                "ros2-solution-node",
                "--ros-args",
                "-p",
                f"solution_file:={solution_path}",
                "-p",
                "publish_period_ms:=1",
                "-p",
                "max_messages:=2",
            )
        self.assertEqual(result.returncode, 0, msg=result.stderr)
        self.assertIn("published_messages=2", result.stdout + result.stderr)
